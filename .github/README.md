# Linux 6.12 for the Galaxy Note9 (SM-N960F, Exynos 9810)

Android Common Kernel `android16-6.12` (6.12.93) with a crownlte port. Experimental.

## Build

```sh
sudo apt install clang-18 lld-18 llvm-18 make bc bison flex libssl-dev libelf-dev
mkdir ../out && cp arch/arm64/configs/crownlte-int6.config ../out/.config
make O=../out ARCH=arm64 LLVM=-18 olddefconfig
make O=../out ARCH=arm64 LLVM=-18 -j$(nproc) Image dtbs

# DTB: built with build-dtb.sh from crownlte-rom-ram, not by kbuild
KSRC=$PWD KO=$PWD/../out ../crownlte-rom-ram/build-dtb.sh \
  arch/arm64/boot/dts/exynos/exynos9810-crownlte-romram-v23a-int6.dts int6.dtb
```

Output: `../out/arch/arm64/boot/Image` and `int6.dtb`. Keep the output directory outside
the source tree; the Mali driver does not build with `O=out` inside it.

## Branches

- `main`: integration 6, tested on hardware (LineageOS boots, display, GPU, touch, Wi-Fi).
  Current status: [STATUS.md](https://github.com/dotcomdomain/crownlte-port-notes/blob/main/STATUS.md).
- `crownlte-*`: feature branches (BT, suspend, C2 idle, audio). They build the same way.
  Notes are in `Documentation/crownlte/`.

## Boot

Boot it from RECOVERY with [crownlte-boot-shim](https://github.com/dotcomdomain/crownlte-boot-shim) and
[crownlte-rom-ram](https://github.com/dotcomdomain/crownlte-rom-ram). Do not flash it to BOOT.

License: GPL-2.0, except `firmware/exynos9810_acpm_tmu.fw` (Samsung).
