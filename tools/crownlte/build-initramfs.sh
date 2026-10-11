#!/bin/bash
set -euo pipefail
root=/root/kernel-upgrade
out="$root/artifacts/diagnostic-initramfs"
mkdir -p "$out/rootfs"
aarch64-linux-gnu-gcc -static -Os -Wall -Wextra -Werror \
  "$root/android16-6.12-port/tools/crownlte/diagnostic-init.c" \
  -o "$out/rootfs/init"
chmod 0755 "$out/rootfs/init"
touch -d @0 "$out/rootfs/init" "$out/rootfs"
(cd "$out/rootfs"; find . -print0 | LC_ALL=C sort -z | \
  cpio --null -o --format=newc --reproducible --owner=0:0) | \
  gzip -n > "$out/diagnostic-initramfs.cpio.gz"
aarch64-linux-gnu-readelf -h "$out/rootfs/init"
if aarch64-linux-gnu-readelf -l "$out/rootfs/init" | grep -q INTERP; then
  echo 'Diagnostic init unexpectedly requires a dynamic linker' >&2
  exit 1
fi
gzip -cd "$out/diagnostic-initramfs.cpio.gz" | cpio -itv
sha256sum "$out/rootfs/init" "$out/diagnostic-initramfs.cpio.gz"
