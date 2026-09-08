/*
 * rec3g — screen capture for a jailbroken iPhone 3G (iOS 4.2.1, armv6).
 *
 * A MobileSubstrate tweak loaded into SpringBoard. It idles blocked in accept()
 * on loopback and, when a client connects, streams raw framebuffer rows to it.
 *
 * WHY THIS LIVES INSIDE SPRINGBOARD, which is the whole story:
 * from any other process IOMobileFramebufferGetLayerDefaultSurface hands back a
 * surface that is frozen — it returns a real image only in the instant after
 * SpringBoard starts, and never updates again. Measured directly: with the
 * phone sitting on the home screen, an outside process read a lock screen from
 * 27 minutes earlier, while this same code inside SpringBoard read the live
 * clock. Veency reaches the display through CAWindowServer, whose
 * +serverIfRunning is nil outside the window server, and Display Recorder was a
 * tweak too. None of them are standalone daemons, and that is not a
 * coincidence — it is the only place the screen is legible.
 *
 * Everything else follows from the phone: a 412 MHz ARM11 and 128 MB of RAM.
 *
 *   Frames are never compressed. Measured throughput, this device:
 *     ssh pipe        1.4 MB/s   (crypto in software)
 *     write to /var   5.4 MB/s
 *     raw TCP         10.4 MB/s  (usbmuxd carries it over the cable)
 *   A 320x480 BGRA frame is 614 KB, so only the raw socket carries 15 fps at
 *   all, and there are no cycles for an encoder besides.
 *
 *   Even 10.4 MB/s is 16.9 whole frames a second, so frames are compared a row
 *   at a time and only changed rows are sent, as runs of consecutive rows. A
 *   swipe touching 40 rows costs 51 KB instead of 614 KB; a still screen costs
 *   nothing. The pixels stay raw, there are just fewer of them.
 *
 *   A ring buffer sits between capture and socket, because a full-screen change
 *   is a 59 ms burst against a 66 ms frame budget. When it is full the frame is
 *   dropped rather than waited on — and a dropped frame deliberately does not
 *   advance the comparison baseline, so its rows go out with the next frame
 *   instead of being lost, which would leave the client's canvas wrong forever.
 *
 * It listens on loopback only. usbmuxd bridges the cable; the wifi network
 * never reaches it, and a live view of the screen should not be one open port
 * away from the rest of the house.
 *
 * Wire format, little-endian, per connection:
 *   header  "R3G2", width, height, stride, bpp, fps   (6 x uint32)
 *   frame   timestamp_us (uint32), run_count (uint32),
 *           then per run: first_row (uint32), row_count (uint32), pixel rows
 * A run_count of 0 is the once-a-second heartbeat: it times how long the screen
 * held still, and it is what makes a departed client show up as a write error.
 * The stream ends at EOF, never at a marker.
 *
 * Build (AirBuild Legacy's GCC 4.2.1, on the phone):
 *   . /etc/profile.d/airbuild.sh
 *   gcc $IOS_CFLAGS -O2 -dynamiclib -nostartfiles $SDKROOT/usr/lib/dylib1.o \
 *       -o rec3g.dylib rec3gsb.c $IOS_LDFLAGS && ldid -S rec3g.dylib
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <dlfcn.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <mach/mach.h>
#include <signal.h>
#include <stdarg.h>
#include <CoreFoundation/CoreFoundation.h>

#define PORT 5999
#define FPS 15
/* 2 MB, not the 4 a desktop would take: this is SpringBoard's address space on
 * a 128 MB device, and Veency died of exactly this. Three whole frames is
 * enough to absorb a burst. */
#define RING_BYTES (2u * 1024 * 1024)
#define LOG_PATH "/tmp/rec3g.log"

typedef mach_port_t io_service_t;

static int (*p_LayerDefaultSurface)(void *, int, void **);
static void *(*p_GetBaseAddress)(void *);
static int (*p_Lock)(void *, unsigned);
static int (*p_Unlock)(void *);
static void (*p_Flush)(void *);
static size_t (*p_GetWidth)(void *);
static size_t (*p_GetHeight)(void *);
static size_t (*p_GetBytesPerRow)(void *);
static void *(*p_SurfaceCreate)(CFDictionaryRef);
static void (*p_RenderDisplay)(int, CFStringRef, void *, int, int);

static FILE *logfile;

/* Ring buffer, filled by the capture loop and drained by the sender thread. */
static unsigned char *ring;
static size_t ring_cap, ring_head, ring_tail;
static int ring_closed, sender_error;
static pthread_mutex_t ring_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t ring_filled = PTHREAD_COND_INITIALIZER;

static void note(const char *format, ...)
{
	if (!logfile)
		return;
	va_list args;
	va_start(args, format);
	vfprintf(logfile, format, args);
	va_end(args);
	fputc('\n', logfile);
	fflush(logfile);
}

static unsigned long long now_us(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (unsigned long long)tv.tv_sec * 1000000ull + tv.tv_usec;
}

static void put32(unsigned char *p, unsigned v)
{
	p[0] = v & 0xff;
	p[1] = (v >> 8) & 0xff;
	p[2] = (v >> 16) & 0xff;
	p[3] = (v >> 24) & 0xff;
}

/* Ask for the backlight, without touching Objective-C.
 *
 * Driving SpringBoard's own screen controller from this thread was tried and is
 * not worth it: sending a selector it does not implement aborted the process,
 * and messaging its UI objects off the main thread wedged the capture loop so
 * accept() never came back. This C call cannot do either. It has not been
 * observed to actually light a dark screen, so a recording still expects the
 * screen to be on — but it costs nothing and is safe from this thread. */
static void wake_display(void)
{
	static void (*set_backlight)(float);
	static int looked_up;

	if (!looked_up) {
		looked_up = 1;
		void *gs = dlopen("/System/Library/PrivateFrameworks/GraphicsServices.framework/"
				  "GraphicsServices", RTLD_LAZY);
		if (gs)
			set_backlight = dlsym(gs, "GSEventSetBacklightLevel");
	}
	if (set_backlight)
		set_backlight(1.0f);
}

static size_t ring_used_locked(void)
{
	return (ring_head + ring_cap - ring_tail) % ring_cap;
}

/* Returns 0 when the frame did not fit, which the caller treats as a drop. */
static int ring_push(const unsigned char *data, size_t len)
{
	pthread_mutex_lock(&ring_lock);
	if (sender_error || ring_cap - ring_used_locked() - 1 < len) {
		pthread_mutex_unlock(&ring_lock);
		return 0;
	}
	size_t first = ring_cap - ring_head;
	if (first > len)
		first = len;
	memcpy(ring + ring_head, data, first);
	if (len > first)
		memcpy(ring, data + first, len - first);
	ring_head = (ring_head + len) % ring_cap;
	pthread_cond_signal(&ring_filled);
	pthread_mutex_unlock(&ring_lock);
	return 1;
}

static void *sender(void *arg)
{
	int fd = *(int *)arg;
	for (;;) {
		pthread_mutex_lock(&ring_lock);
		while (ring_used_locked() == 0 && !ring_closed)
			pthread_cond_wait(&ring_filled, &ring_lock);
		size_t used = ring_used_locked();
		if (used == 0 && ring_closed) {
			pthread_mutex_unlock(&ring_lock);
			return NULL;
		}
		size_t tail = ring_tail;
		size_t chunk = ring_cap - tail;
		if (chunk > used)
			chunk = used;
		pthread_mutex_unlock(&ring_lock);

		/* Safe outside the lock: the producer never writes past ring_tail. */
		ssize_t n = write(fd, ring + tail, chunk);
		if (n <= 0) {
			pthread_mutex_lock(&ring_lock);
			sender_error = 1;
			pthread_mutex_unlock(&ring_lock);
			return NULL;
		}
		pthread_mutex_lock(&ring_lock);
		ring_tail = (ring_tail + (size_t)n) % ring_cap;
		pthread_mutex_unlock(&ring_lock);
	}
}

struct display {
	void *surface;		/* ours; the render server draws the screen into it */
	size_t width, height;
	size_t src_stride;	/* that surface: BGRA, 4 bytes a pixel */
	size_t stride;		/* what goes on the wire: RGB565, 2 bytes a pixel */
	size_t frame_bytes;
};

/* Convert one BGRA frame to RGB565 while it is read.
 *
 * Halving the pixel costs about 4 ms of ARM11 per frame and buys the thing the
 * recording actually runs out of: at 15 fps a full-screen change is 9.2 MB/s in
 * BGRA against a 10.4 MB/s channel, which drops frames the moment anything
 * moves. In RGB565 the same change is 4.6 MB/s. The panel is 18-bit and the
 * result is a video, not a screenshot, so the two low bits are not missed —
 * and every comparison downstream gets twice as cheap as well.
 */
static void pack_565(const unsigned char *src, size_t src_stride, unsigned char *dst,
		     size_t dst_stride, size_t width, size_t height)
{
	size_t y;
	for (y = 0; y < height; y++) {
		const unsigned char *in = src + y * src_stride;
		unsigned short *out = (unsigned short *)(dst + y * dst_stride);
		size_t x;
		for (x = 0; x < width; x++, in += 4)
			out[x] = (unsigned short)(((in[2] & 0xf8) << 8) |
						  ((in[1] & 0xfc) << 3) | (in[0] >> 3));
	}
}

static void set_number(CFMutableDictionaryRef dict, CFStringRef key, int value)
{
	if (!key)
		return;
	CFNumberRef number = CFNumberCreate(NULL, kCFNumberIntType, &value);
	CFDictionarySetValue(dict, key, number);
	CFRelease(number);
}

static int open_display(struct display *d)
{
	void *iokit = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_LAZY);
	void *iomfb = dlopen("/System/Library/PrivateFrameworks/IOMobileFramebuffer.framework/"
			     "IOMobileFramebuffer", RTLD_LAZY);
	void *csurf = dlopen("/System/Library/PrivateFrameworks/CoreSurface.framework/CoreSurface",
			     RTLD_LAZY);
	void *quartz = dlopen("/System/Library/Frameworks/QuartzCore.framework/QuartzCore",
			      RTLD_LAZY);
	if (!iokit || !iomfb || !csurf || !quartz) {
		note("rec3g: dlopen failed");
		return -1;
	}

	void *(*Matching)(const char *) = dlsym(iokit, "IOServiceMatching");
	io_service_t (*GetService)(mach_port_t, void *) =
		dlsym(iokit, "IOServiceGetMatchingService");
	int (*Open)(io_service_t, mach_port_t, unsigned, void **) =
		dlsym(iomfb, "IOMobileFramebufferOpen");
	p_LayerDefaultSurface = dlsym(iomfb, "IOMobileFramebufferGetLayerDefaultSurface");
	p_GetBaseAddress = dlsym(csurf, "CoreSurfaceBufferGetBaseAddress");
	p_Lock = dlsym(csurf, "CoreSurfaceBufferLock");
	p_Unlock = dlsym(csurf, "CoreSurfaceBufferUnlock");
	p_Flush = dlsym(csurf, "CoreSurfaceBufferFlushProcessorCaches");
	p_GetWidth = dlsym(csurf, "CoreSurfaceBufferGetWidth");
	p_GetHeight = dlsym(csurf, "CoreSurfaceBufferGetHeight");
	p_GetBytesPerRow = dlsym(csurf, "CoreSurfaceBufferGetBytesPerRow");
	p_SurfaceCreate = dlsym(csurf, "CoreSurfaceBufferCreate");
	p_RenderDisplay = dlsym(quartz, "CARenderServerRenderDisplay");
	if (!Matching || !GetService || !Open || !p_LayerDefaultSurface || !p_GetBaseAddress ||
	    !p_Lock || !p_Unlock || !p_GetWidth || !p_GetHeight || !p_GetBytesPerRow ||
	    !p_SurfaceCreate || !p_RenderDisplay) {
		note("rec3g: missing a required symbol");
		return -1;
	}

	/* Geometry comes from the real framebuffer, even though we do not read it:
	 * its numbers are right even when its contents are stale. */
	static const char *names[] = { "AppleH1CLCD", "AppleCLCD", "AppleM2CLCD",
				       "AppleMobileCLCD", "IOMobileFramebuffer", NULL };
	io_service_t service = 0;
	void *connection = NULL, *fb_surface = NULL;
	int i;
	for (i = 0; names[i] && !service; i++)
		service = GetService(0, Matching(names[i]));
	if (!service || Open(service, mach_task_self(), 0, &connection) || !connection ||
	    p_LayerDefaultSurface(connection, 0, &fb_surface) || !fb_surface) {
		note("rec3g: cannot reach the framebuffer for its geometry");
		return -1;
	}
	d->width = p_GetWidth(fb_surface);
	d->height = p_GetHeight(fb_surface);
	if (!d->width || !d->height) {
		note("rec3g: bad geometry");
		return -1;
	}
	d->src_stride = d->width * 4;
	d->stride = d->width * 2;
	d->frame_bytes = d->stride * d->height;

	/* Our own surface for the render server to draw into.
	 *
	 * The framebuffer's own layer-0 surface is useless from here: it holds
	 * whatever was on screen when SpringBoard started and never changes again,
	 * so recordings caught the boot animation and then 40 seconds of a still
	 * image no matter what the phone was doing. CARenderServerRenderDisplay
	 * composites the screen as it is now — it is what the OS itself uses for
	 * screenshots and app-switcher thumbnails. */
	CFStringRef *key_global = dlsym(csurf, "kCoreSurfaceBufferGlobal");
	CFStringRef *key_region = dlsym(csurf, "kCoreSurfaceBufferMemoryRegion");
	CFStringRef *key_pitch = dlsym(csurf, "kCoreSurfaceBufferPitch");
	CFStringRef *key_width = dlsym(csurf, "kCoreSurfaceBufferWidth");
	CFStringRef *key_height = dlsym(csurf, "kCoreSurfaceBufferHeight");
	CFStringRef *key_format = dlsym(csurf, "kCoreSurfaceBufferPixelFormat");
	CFStringRef *key_size = dlsym(csurf, "kCoreSurfaceBufferAllocSize");
	if (!key_global || !key_region || !key_pitch || !key_width || !key_height ||
	    !key_format || !key_size) {
		note("rec3g: CoreSurface dictionary keys are missing");
		return -1;
	}

	CFMutableDictionaryRef dict = CFDictionaryCreateMutable(
		NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
	CFDictionarySetValue(dict, *key_global, kCFBooleanTrue);
	CFDictionarySetValue(dict, *key_region, CFSTR("PurpleGFXMem"));
	set_number(dict, *key_pitch, (int)d->src_stride);
	set_number(dict, *key_width, (int)d->width);
	set_number(dict, *key_height, (int)d->height);
	set_number(dict, *key_format, 0x42475241); /* 'BGRA' */
	set_number(dict, *key_size, (int)(d->src_stride * d->height));
	d->surface = p_SurfaceCreate(dict);
	CFRelease(dict);
	if (!d->surface) {
		note("rec3g: CoreSurfaceBufferCreate failed");
		return -1;
	}
	return 0;
}

/* Stream the display to one connected client until it goes away. */
static void serve(int fd, struct display *d)
{
	unsigned char *current = malloc(d->frame_bytes);
	unsigned char *previous = malloc(d->frame_bytes);
	/* Worst case is every row changed, each in its own run. */
	unsigned char *packet = malloc(8 + d->height * (8 + d->stride));
	unsigned *run_first = malloc(d->height * sizeof *run_first);
	unsigned *run_rows = malloc(d->height * sizeof *run_rows);
	ring = malloc(RING_BYTES);
	if (!current || !previous || !packet || !run_first || !run_rows || !ring) {
		note("rec3g: out of memory");
		goto done;
	}
	ring_cap = RING_BYTES;
	ring_head = ring_tail = 0;
	ring_closed = sender_error = 0;

	/* No valid frame yet, so the first comparison sends the whole screen. */
	memset(previous, 0xff, d->frame_bytes);

	unsigned char header[24];
	memcpy(header, "R3G2", 4);
	put32(header + 4, (unsigned)d->width);
	put32(header + 8, (unsigned)d->height);
	put32(header + 12, (unsigned)d->stride);
	put32(header + 16, (unsigned)(d->stride / d->width));
	put32(header + 20, FPS);
	if (write(fd, header, sizeof header) != (ssize_t)sizeof header)
		goto done;

	pthread_t thread;
	if (pthread_create(&thread, NULL, sender, &fd) != 0) {
		note("rec3g: pthread_create failed");
		goto done;
	}

	wake_display();

	const unsigned long long interval = 1000000ull / FPS;
	const unsigned long long started = now_us();
	unsigned long long deadline = started, last_send = started, last_wake = started;
	unsigned long captured = 0, sent = 0, dropped = 0;
	unsigned long long payload = 0;
	int i;

	for (;;) {
		pthread_mutex_lock(&ring_lock);
		int broken = sender_error;
		pthread_mutex_unlock(&ring_lock);
		if (broken)
			break;

		/* Auto-lock would dim the screen out from under a long take. */
		if (now_us() - last_wake >= 20000000ull) {
			wake_display();
			last_wake = now_us();
		}

		/* Have the render server composite the screen as it is right now,
		 * then pack it to RGB565 while it is locked. */
		p_Lock(d->surface, 3);
		p_RenderDisplay(0, CFSTR("LCD"), d->surface, 0, 0);
		unsigned char *base = p_GetBaseAddress(d->surface);
		if (base) {
			if (p_Flush)
				p_Flush(d->surface);
			pack_565(base, d->src_stride, current, d->stride, d->width, d->height);
		}
		p_Unlock(d->surface);

		/* A sleeping display has no backing address. That is a state, not a
		 * failure: hold the connection and keep looking. */
		if (!base)
			goto pace;
		captured++;

		/* Pack the changed rows as runs of consecutive rows. */
		unsigned runs = 0;
		size_t row = 0;
		while (row < d->height) {
			if (!memcmp(current + row * d->stride, previous + row * d->stride,
				    d->stride)) {
				row++;
				continue;
			}
			size_t first = row;
			while (row < d->height && memcmp(current + row * d->stride,
							 previous + row * d->stride, d->stride))
				row++;
			run_first[runs] = (unsigned)first;
			run_rows[runs] = (unsigned)(row - first);
			runs++;
		}

		if (runs) {
			size_t at = 8;
			put32(packet, (unsigned)(now_us() - started));
			put32(packet + 4, runs);
			for (i = 0; i < (int)runs; i++) {
				size_t bytes = (size_t)run_rows[i] * d->stride;
				put32(packet + at, run_first[i]);
				put32(packet + at + 4, run_rows[i]);
				memcpy(packet + at + 8, current + (size_t)run_first[i] * d->stride,
				       bytes);
				at += 8 + bytes;
			}
			if (ring_push(packet, at)) {
				sent++;
				payload += at;
				unsigned char *swap = previous;
				previous = current;
				current = swap;
			} else {
				/* Keep the baseline: these rows ride along next frame. */
				dropped++;
			}
			last_send = now_us();
		} else if (now_us() - last_send >= 1000000ull) {
			/* A still screen sends nothing, and a server that only ever
			 * writes would never notice the client had gone. An empty frame
			 * once a second fails the write when nobody is listening, and
			 * carries the time the screen held still. */
			unsigned char beat[8];
			put32(beat, (unsigned)(now_us() - started));
			put32(beat + 4, 0);
			ring_push(beat, sizeof beat);
			last_send = now_us();
		}

pace:
		{
			deadline += interval;
			unsigned long long now = now_us();
			if (now < deadline)
				usleep((useconds_t)(deadline - now));
			else
				deadline = now; /* Behind: don't catch up in a burst. */
		}
	}

	pthread_mutex_lock(&ring_lock);
	ring_closed = 1;
	pthread_cond_signal(&ring_filled);
	pthread_mutex_unlock(&ring_lock);
	pthread_join(thread, NULL);

	double elapsed = (double)(now_us() - started) / 1000000.0;
	note("rec3g: %.1fs, %lu captured, %lu sent, %lu dropped, %.1f MB (%.1f MB/s)",
	     elapsed, captured, sent, dropped, payload / 1e6,
	     elapsed > 0 ? payload / 1e6 / elapsed : 0.0);

done:
	free(current);
	free(previous);
	free(packet);
	free(run_first);
	free(run_rows);
	free(ring);
	ring = NULL;
	ring_cap = 0;
}

static void *server_thread(void *unused)
{
	(void)unused;
	/* Let SpringBoard finish coming up before touching the display. */
	sleep(8);

	logfile = fopen(LOG_PATH, "a");
	if (logfile)
		setvbuf(logfile, NULL, _IOLBF, 0);

	struct display display;
	memset(&display, 0, sizeof display);
	if (open_display(&display))
		return NULL;

	int listener = socket(AF_INET, SOCK_STREAM, 0);
	if (listener < 0) {
		note("rec3g: socket failed");
		return NULL;
	}
	int one = 1;
	setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof addr);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);
	/* Loopback only: usbmuxd bridges the cable, the network never gets in. */
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (bind(listener, (struct sockaddr *)&addr, sizeof addr) || listen(listener, 1)) {
		note("rec3g: bind/listen failed: %s", strerror(errno));
		close(listener);
		return NULL;
	}
	note("rec3g: %zux%zu, idle on 127.0.0.1:%d at %d fps",
	     display.width, display.height, PORT, FPS);

	for (;;) {
		int fd = accept(listener, NULL, NULL);
		if (fd < 0) {
			if (errno == EINTR)
				continue;
			note("rec3g: accept failed: %s", strerror(errno));
			break;
		}
		setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
		serve(fd, &display);
		/* Half-close first so the client sees EOF rather than a stalled read. */
		shutdown(fd, SHUT_RDWR);
		close(fd);
	}
	close(listener);
	return NULL;
}

__attribute__((constructor))
static void rec3g_loaded(void)
{
	/* A client that hangs up must be a write error, not SpringBoard's death. */
	signal(SIGPIPE, SIG_IGN);

	pthread_t thread;
	if (pthread_create(&thread, NULL, server_thread, NULL) == 0)
		pthread_detach(thread);
}
