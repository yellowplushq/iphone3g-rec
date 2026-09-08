# rec3g

Screen capture for a jailbroken **iPhone 3G** (iOS 4.2.1, armv6).

A MobileSubstrate tweak loaded into SpringBoard streams the live framebuffer
over USB. A Python CLI on the host installs that tweak, records the stream, and
encodes it to MP4.

The capture has to live inside SpringBoard. From any other process,
`IOMobileFramebufferGetLayerDefaultSurface` returns a frozen image — the
surface that existed when SpringBoard started. Veency and Display Recorder are
tweaks for the same reason.

## Requirements

**Phone**

- Jailbroken iPhone 3G on iOS 4.2.1
- MobileSubstrate
- AirBuild Legacy (GCC 4.2.1 on-device), with its environment installed
  (`Settings > Environment > Install`)
- OpenSSH (default root password is `alpine`)

**Host**

- Python 3
- `ffmpeg`
- `sshpass`
- `libimobiledevice` (`iproxy`)

## Usage

```sh
./rec3g install                     # build on the phone, install, respring
./rec3g record -o demo.mp4          # record until Ctrl-C
./rec3g record -o demo.mp4 -s 30    # or for a fixed 30 seconds
./rec3g uninstall
```

Keep the screen awake while recording. A sleeping display has no backing
address, so the stream either stays black or freezes on the last frame.

Pass `--udid` and `--password` if the device is not the default UDID or the
root password is not `alpine`.

## How it works

The tweak (`rec3gsb.c`) is injected into SpringBoard and listens on
`127.0.0.1:5999`. `usbmuxd` (via `iproxy`) bridges that port over the cable;
nothing is written to the phone's flash and the Wi-Fi network never sees the
stream.

Frames are packed to RGB565 and compared a row at a time. Only changed rows are
sent. A still screen costs a one-byte-rate heartbeat once a second.

Wire format, little-endian, per connection:

- header: `"R3G2"`, width, height, stride, bpp, fps (6 × `uint32`)
- frame: `timestamp_us`, `run_count`, then per run: `first_row`, `row_count`,
  pixel rows

A `run_count` of 0 is the heartbeat. The stream ends at EOF.

## Build (on the phone)

`rec3g install` does this for you. Equivalent manual build with AirBuild Legacy:

```sh
. /etc/profile.d/airbuild.sh
gcc $IOS_CFLAGS -O2 -dynamiclib -nostartfiles $SDKROOT/usr/lib/dylib1.o \
    -o rec3g.dylib rec3gsb.c $IOS_LDFLAGS -framework CoreFoundation
ldid -S rec3g.dylib
```

Install `rec3g.dylib` and `rec3g.plist` into
`/Library/MobileSubstrate/DynamicLibraries/` and respring.

## License

MIT. See [LICENSE](LICENSE).
