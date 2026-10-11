#!/bin/bash
# Build the isolated source-only clock foundation. Never accesses a phone.
set -euo pipefail
source_root=$(cd "$(dirname "$0")/../.." && pwd)
: "${KERNEL_OUT:?Set KERNEL_OUT to a separate initialized diagnostic output tree}"
cd "$source_root"
scripts/config --file "$KERNEL_OUT/.config" -e EXYNOS9810_LEGACY_CAL -d ECT_DUMP -e WERROR
make O="$KERNEL_OUT" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- LLVM=-18 olddefconfig
make O="$KERNEL_OUT" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- LLVM=-18 -j2 Image dtbs
