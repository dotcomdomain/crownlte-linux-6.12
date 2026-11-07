/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos 9810 MCT tick broadcast device (drivers/clocksource/exynos9810-mct-bc.c)
 */
#ifndef __SOC_SAMSUNG_EXYNOS9810_MCT_BC_H
#define __SOC_SAMSUNG_EXYNOS9810_MCT_BC_H

#include <linux/cpumask.h>
#include <linux/types.h>

struct exynos9810_mct_bc_stats {
	u64 irqs;		/* comparator interrupts, self-check included */
	u64 programmed;		/* set_next_event calls */
	u64 etime;		/* set_next_event returned -ETIME */
	u64 write_timeouts;	/* MCT write status never confirmed */
};

/* Per-CPU MCT local timer (L0-L7), DT samsung,mct-local-timers. */
struct exynos9810_mct_local_stats {
	u64 irqs;		/* local timer interrupts on this CPU */
	u64 write_timeouts;	/* local register write never confirmed */
	bool active;		/* the timer runs this CPU's tick now */
	bool failed;		/* latched failed on this boot */
};

/*
 * Called by the local timer fault work (process context, CPU0) for a
 * timer that was latched failed after the takeover. @fault runs first
 * (record the fault, LITTLE C2 off), then the CPU's tick device is
 * replaced by its arch timer; @fatal only if that replacement failed.
 */
struct exynos9810_mct_local_ops {
	void (*fault)(unsigned int cpu, const char *why);
	void (*fatal)(unsigned int cpu, const char *why);
};

#ifdef CONFIG_EXYNOS9810_MCT_BROADCAST
/* True once the MCT passed its self-check and is the broadcast device. */
bool exynos9810_mct_bc_ready(void);
/* Boot time (ms) the MCT became the broadcast device, 0 if never. */
u64 exynos9810_mct_bc_ready_ms(void);
void exynos9810_mct_bc_get_stats(struct exynos9810_mct_bc_stats *s);
/*
 * Once per boot, process context: self-check and register the MCT local
 * timer of every online CPU in @want (CPU0 is never taken) as that CPU's
 * tick device. @active gets the CPUs that tick on it afterwards. NULL if
 * at least one did, else why not. Every other CPU keeps the arch timer +
 * broadcast. The caller must have persisted the local-timer session first.
 */
const char *exynos9810_mct_local_takeover(const struct cpumask *want,
				const struct exynos9810_mct_local_ops *ops,
				struct cpumask *active);
/*
 * True if @cpu's tick runs on its MCT local timer now and the timer has
 * not failed, i.e. a C2 period of @cpu is woken by that timer's own
 * interrupt and never by the broadcast. Turns false, on that CPU, at the
 * moment a write or acknowledge fails. Safe from the idle path.
 */
bool exynos9810_mct_local_active(unsigned int cpu);
/* Local timers latched failed on this boot (any context). */
unsigned int exynos9810_mct_local_faults(void);
void exynos9810_mct_local_get_stats(unsigned int cpu,
				    struct exynos9810_mct_local_stats *s);
#else
static inline const char *
exynos9810_mct_local_takeover(const struct cpumask *want,
			      const struct exynos9810_mct_local_ops *ops,
			      struct cpumask *active)
{
	cpumask_clear(active);
	return "no MCT broadcast driver";
}

static inline bool exynos9810_mct_local_active(unsigned int cpu)
{
	return false;
}

static inline unsigned int exynos9810_mct_local_faults(void)
{
	return 0;
}

static inline void exynos9810_mct_local_get_stats(unsigned int cpu,
					struct exynos9810_mct_local_stats *s)
{
	*s = (struct exynos9810_mct_local_stats){ };
}

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
