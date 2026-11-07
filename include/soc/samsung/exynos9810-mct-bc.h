/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos 9810 MCT tick broadcast device (drivers/clocksource/exynos9810-mct-bc.c)
 */
#ifndef __SOC_SAMSUNG_EXYNOS9810_MCT_BC_H
#define __SOC_SAMSUNG_EXYNOS9810_MCT_BC_H

#include <linux/types.h>

struct exynos9810_mct_bc_stats {
	u64 irqs;		/* comparator interrupts, self-check included */
	u64 programmed;		/* set_next_event calls */
	u64 etime;		/* set_next_event returned -ETIME */
	u64 write_timeouts;	/* MCT write status never confirmed */
};

#ifdef CONFIG_EXYNOS9810_MCT_BROADCAST
/* True once the MCT passed its self-check and is the broadcast device. */
bool exynos9810_mct_bc_ready(void);
/* Boot time (ms) the MCT became the broadcast device, 0 if never. */
u64 exynos9810_mct_bc_ready_ms(void);
void exynos9810_mct_bc_get_stats(struct exynos9810_mct_bc_stats *s);
#else
static inline bool exynos9810_mct_bc_ready(void)
{
	return false;
}

static inline u64 exynos9810_mct_bc_ready_ms(void)
{
	return 0;
}

static inline void exynos9810_mct_bc_get_stats(struct exynos9810_mct_bc_stats *s)
{
	*s = (struct exynos9810_mct_bc_stats){ };
}
#endif

#endif /* __SOC_SAMSUNG_EXYNOS9810_MCT_BC_H */
