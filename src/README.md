# src

`ak_rtsp_demo.c` is built against the Anyka AK3918 SDK. The SDK headers and
shared libraries are **not** included (proprietary) — extract them from your
own camera / the vendor SDK.

## Build
```sh
sh build.sh        # -> ak_rtsp_demo_v104
```

Edit `build.sh` to point at your toolchain and SDK include/lib paths.
Toolchain: `arm-anykav200-linux-uclibcgnueabi` (uClibc 0.9.33, `ld-uClibc.so.0`).

## Extra libraries to ship with the firmware
These four are needed by the binary but are **not** in the stock `/usr/lib`,
so copy them into the repacked `/usr/lib`:

- `libakae.so`
- `libplat_common_one.so`
- `libplat_mem.so`
- `libplat_osal.so`

The system C libraries (`libc.so.0`, `libpthread.so.0`, `librt.so.0`,
`libdl.so.0`, `libgcc_s.so.1`) are already in `/lib` on the camera.
