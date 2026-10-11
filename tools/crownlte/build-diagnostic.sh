#!/bin/bash
set -euo pipefail
export ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- LLVM=-18
out=${OUT:-/root/kernel-upgrade/build/diag-6.12}
mkdir -p "$out"
if [ ! -f "$out/.config" ]; then
  make O="$out" allnoconfig
fi
cfg() { scripts/config --file "$out/.config" "$@"; }
for symbol in 64BIT MMU SMP ARCH_EXYNOS OF EXPERT PRINTK BUG KALLSYMS \
  BINFMT_ELF BINFMT_SCRIPT BLK_DEV_INITRD BLOCK TTY SERIAL_SAMSUNG \
  SERIAL_SAMSUNG_CONSOLE PINCTRL PINCTRL_EXYNOS GPIO_CDEV EXYNOS_PMU \
  EXYNOS_CHIPID EXYNOS9810_DIAGNOSTIC PROC_FS SYSFS TMPFS DEVTMPFS \
  DEVTMPFS_MOUNT UNIX NET INET IPV6 PSTORE PSTORE_RAM PSTORE_CONSOLE \
  SCSI BLK_DEV_SD SCSI_UFSHCD SCSI_UFSHCD_PLATFORM SCSI_UFS_EXYNOS9810 \
  PHY_EXYNOS9810_USBDRD USB_SUPPORT USB_GADGET USB_DWC3 USB_DWC3_GADGET \
  USB_CONFIGFS USB_CONFIGFS_ACM USB_CONFIGFS_NCM USB_CONFIGFS_F_FS; do
  cfg --enable "$symbol"
done
cfg --disable SCSI_UFS_CRYPTO
cfg --disable RANDOMIZE_BASE
cfg --enable ARM64_4K_PAGES
cfg --enable ARM64_VA_BITS_48
cfg --enable ARM64_PA_BITS_48
cfg --disable WERROR
cfg --set-str LOCALVERSION "-crownlte-diag"
make O="$out" olddefconfig
make -j${JOBS:-4} O="$out" Image exynos/exynos9810-crownlte-diag.dtb
sha256sum "$out/arch/arm64/boot/Image" "$out/arch/arm64/boot/dts/exynos/exynos9810-crownlte-diag.dtb"
