# SPDX-License-Identifier: GPL-2.0

obj-$(CONFIG_EXYNOS_ASV_ARM)	+= exynos5422-asv.o
obj-$(CONFIG_EXYNOS_CHIPID)	+= exynos_chipid.o
exynos_chipid-y			+= exynos-chipid.o exynos-asv.o

obj-$(CONFIG_EXYNOS_USI)	+= exynos-usi.o

obj-$(CONFIG_EXYNOS_PMU)	+= exynos-pmu.o

obj-$(CONFIG_EXYNOS_PMU_ARM_DRIVERS)	+= exynos3250-pmu.o exynos4-pmu.o \
					exynos5250-pmu.o exynos5420-pmu.o
obj-$(CONFIG_EXYNOS_REGULATOR_COUPLER) += exynos-regulator-coupler.o

obj-$(CONFIG_SAMSUNG_PM_CHECK)	+= s3c-pm-check.o

obj-$(CONFIG_EXYNOS9810_DIAGNOSTIC) += s9p-clkgate.o

obj-$(CONFIG_EXYNOS9810_DIAGNOSTIC_MUIC) += exynos9810-max77705-muic.o
obj-$(CONFIG_EXYNOS9810_LEGACY_CAL) += cal-if/ acpm/ ect_parser.o exynos9810-cal-smc.o

obj-$(CONFIG_EXYNOS9810_DIAGNOSTIC_RESTART) += exynos9810-diagnostic-restart.o
obj-$(CONFIG_EXYNOS9810_PM) += exynos9810-pm.o
