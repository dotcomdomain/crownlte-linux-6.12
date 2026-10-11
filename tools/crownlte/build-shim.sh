#!/bin/bash
set -euo pipefail
root=/root/kernel-upgrade
src="$root/boot-shim/crownlte"
kernel="$root/build/usb-diag-6.12"
out="$root/artifacts/crownlte-display-free-shim"
python3 "$root/inspect-diagnostic-artifacts.py" "$kernel"
mkdir -p "$src/blob" "$out"
cp "$kernel/arch/arm64/boot/Image" "$src/blob/Image"
cp "$kernel/arch/arm64/boot/dts/exynos/exynos9810-crownlte-usb-diag.dtb" "$src/blob/dtb"
cp "$root/artifacts/diagnostic-initramfs/diagnostic-initramfs.cpio.gz" "$src/blob/ramdisk"
cd "$src"
make ARCH=aarch64 CROSS_COMPILE=aarch64-linux-gnu- crownlte_defconfig
if ! grep -qx '# CONFIG_SIMPLE_FB is not set' .config; then
  echo 'Default shim unexpectedly enables display output' >&2
  exit 1
fi
if grep -qx 'CONFIG_CROWNLTE_FRAMEBUFFER_CONSOLE=y' .config; then
  echo 'Default shim unexpectedly enables framebuffer MMIO' >&2
  exit 1
fi
make ARCH=aarch64 CROSS_COMPILE=aarch64-linux-gnu- -j1
cp uniLoader uniLoader.o .config CROWNLTE-PROVENANCE.txt "$out/"
sha256sum "$out/uniLoader" "$out/uniLoader.o" "$out/.config"
echo 'Loader binary built only. No Android boot image packed and no phone writes.'
