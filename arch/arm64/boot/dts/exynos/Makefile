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

# Deliberately resets the phone at ~115 s (watchdog expiry selftest)
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-wdt-selftest-diag.dtb
dtb-$(CONFIG_ARCH_EXYNOS) += exynos9810-crownlte-delayed-ufs-rtc-latch-diag.dtb
