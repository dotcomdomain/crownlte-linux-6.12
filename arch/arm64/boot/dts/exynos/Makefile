# SPDX-License-Identifier: GPL-2.0
subdir-y += google

dtb-$(CONFIG_ARCH_EXYNOS) += \
	exynos5433-tm2.dtb		\
	exynos5433-tm2e.dtb		\
	exynos7-espresso.dtb		\
	exynos7885-jackpotlte.dtb	\
	exynos850-e850-96.dtb		\
	exynosautov9-sadk.dtb		\
	exynosautov920-sadk.dtb

dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-diag.dtb

dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-usb-diag.dtb

dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-real-clock-usb-diag.dtb

dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-diag.dtb
# panel_drv@001 is a literal vendor ABI path (selinux genfscon), keep its name
DTC_FLAGS_exynos9810-crownlte-delayed-ufs-display-s2-diag := -Wno-unit_address_format
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-display-s2-diag.dtb
# stage 2 + the 4.9 fault-display brightness (sends panel commands via DSIM0)
DTC_FLAGS_exynos9810-crownlte-delayed-ufs-display-s2-bright-diag := -Wno-unit_address_format
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-display-s2-bright-diag.dtb

# Deliberately resets the phone at ~115 s (watchdog expiry selftest)
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-wdt-selftest-diag.dtb
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-rtc-latch-diag.dtb
# G3D power-domain trials: PMIC G3D rail request allowed (see the .dts)
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-g3dpd-diag.dtb
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-g3dpd-runtime-diag.dtb
# Trustonic TEE start (q29). Never combine with ROM-in-RAM or userspace TEE clients.
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-tee-diag.dtb
# sec_ts Y771 touchscreen (q32; IC unpowered until LDO35/LDO43 are enabled)
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-touch-diag.dtb
# Wi-Fi stage 1 (PCIe ch0 + BCM4361 + "wifi selftest:")
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-wifi-diag.dtb
# integration-2 combined diagnostic (q45): all verified features at once
DTC_FLAGS_exynos9810-crownlte-delayed-ufs-combined-diag := -Wno-unit_address_format
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-combined-diag.dtb
# combined + kernel-buffer display matrix at 60-80 s (q48)
DTC_FLAGS_exynos9810-crownlte-delayed-ufs-combined-dispmatrix-diag := -Wno-unit_address_format
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-combined-dispmatrix-diag.dtb
