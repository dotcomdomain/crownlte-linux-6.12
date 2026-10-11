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
  DEVTMPFS_MOUNT UNIX NET INET IPV6 MISC_FILESYSTEMS PSTORE PSTORE_RAM PSTORE_CONSOLE \
  SCSI BLK_DEV_SD SCSI_UFSHCD SCSI_UFSHCD_PLATFORM SCSI_UFS_EXYNOS9810 \
  PHY_EXYNOS9810_USBDRD USB_SUPPORT USB_GADGET USB_DWC3 USB_DWC3_GADGET \
  USB_CONFIGFS USB_CONFIGFS_ACM USB_CONFIGFS_NCM USB_CONFIGFS_F_FS; do
  cfg --enable "$symbol"
done
if [ "${FMP:-0}" = 1 ]; then
  for symbol in CRYPTO ARM64_CRYPTO CRYPTO_AES CRYPTO_AES_ARM64 \
    CRYPTO_AES_ARM64_CE CRYPTO_XTS CRYPTO_CTS CRYPTO_SHA256 CRYPTO_SHA512 \
    CRYPTO_HMAC CRYPTO_HKDF BLK_INLINE_ENCRYPTION \
    BLK_INLINE_ENCRYPTION_FALLBACK SCSI_UFS_CRYPTO FS_ENCRYPTION \
    FS_ENCRYPTION_INLINE_CRYPT F2FS_FS F2FS_FS_XATTR F2FS_FS_SECURITY \
    EXT4_FS EXT4_FS_SECURITY EROFS_FS SECURITY SECURITYFS \
    AUDIT SECURITY_SELINUX SECURITY_NETWORK DEFAULT_SECURITY_SELINUX \
    ANDROID_BINDER_IPC ANDROID_BINDERFS MULTIUSER NAMESPACES \
    UTS_NS IPC_NS USER_NS PID_NS NET_NS CGROUPS MEMCG CGROUP_PIDS \
    CGROUP_FREEZER CGROUP_SCHED CGROUP_CPUACCT CGROUP_BPF \
    BLK_CGROUP CFS_BANDWIDTH BPF BPF_SYSCALL BPF_JIT \
    COMPAT FUTEX EPOLL EVENTFD INOTIFY_USER; do
    cfg --enable "$symbol"
  done
  cfg --set-str LSM "landlock,lockdown,yama,integrity,selinux,bpf"
  cfg --set-str ANDROID_BINDER_DEVICES "binder,hwbinder,vndbinder"
else
  cfg --disable SCSI_UFS_CRYPTO
fi
cfg --disable RANDOMIZE_BASE
# scripts/config does not clear other choice selections automatically.
cfg --disable ARM64_16K_PAGES
cfg --disable ARM64_64K_PAGES
cfg --disable ARM64_VA_BITS_52
cfg --disable ARM64_PA_BITS_52
cfg --disable ARM64_FORCE_52BIT
cfg --enable ARM64_4K_PAGES
cfg --enable ARM64_VA_BITS_48
cfg --enable ARM64_PA_BITS_48
cfg --disable WERROR
cfg --set-str LOCALVERSION "-crownlte-diag"
make O="$out" olddefconfig
require() {
  if ! grep -qx "$1" "$out/.config"; then
    printf 'Required resolved configuration absent: %s\n' "$1" >&2
    exit 1
  fi
}
require CONFIG_ARM64_4K_PAGES=y
require CONFIG_ARM64_VA_BITS=48
require CONFIG_ARM64_PA_BITS=48
require CONFIG_PSTORE_RAM=y
if [ "${FMP:-0}" = 1 ]; then
  for symbol in AUDIT SECURITY_SELINUX SCSI_UFS_CRYPTO \
    BLK_INLINE_ENCRYPTION_FALLBACK FS_ENCRYPTION_INLINE_CRYPT \
    F2FS_FS_SECURITY ANDROID_BINDER_IPC COMPAT; do
    require "CONFIG_$symbol=y"
  done
fi
make -j${JOBS:-4} O="$out" Image exynos/exynos9810-crownlte-diag.dtb
sha256sum "$out/arch/arm64/boot/Image" "$out/arch/arm64/boot/dts/exynos/exynos9810-crownlte-diag.dtb"
