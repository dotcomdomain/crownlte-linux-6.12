#include "pmucal_system.h"
#include "pmucal_rae.h"
#include "pmucal_powermode.h"

unsigned int pmucal_sys_powermode[NUM_SYS_POWERDOWN] = {0xffffffff, };

/**
 *  pmucal_system_enter - prepares to enter a system power mode.
 *		        exposed to PWRCAL interface.
 *
 *  @mode: index of system power mode described in pmucal_system.h.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int pmucal_system_enter(int mode)
{
	int ret;

	if (mode != SYS_SICD)
		exynos_ss_pmu(mode, __func__, ESS_FLAG_IN);

	if (mode >= NUM_SYS_POWERDOWN) {
		pr_err("%s %s: mode index(%d) is out of supported range (0~%d).\n",
				PMUCAL_PREFIX, __func__, mode, NUM_SYS_POWERDOWN);
		return -EINVAL;
	}

	if (!pmucal_lpm_list[mode].enter) {
		pr_err("%s %s: there is no sequence element for entering mode(%d).\n",
				PMUCAL_PREFIX, __func__, mode);
		return -ENOENT;
	}

	pmucal_powermode_hint(pmucal_sys_powermode[mode]);

	pmucal_rae_save_seq(pmucal_lpm_list[mode].save, pmucal_lpm_list[mode].num_save);

	ret = pmucal_rae_handle_seq(pmucal_lpm_list[mode].enter,
				pmucal_lpm_list[mode].num_enter);
	if (ret) {
		pr_err("%s %s: error on handling enter sequence. (mode : %d)\n",
				PMUCAL_PREFIX, __func__, mode);
		return ret;
	}

	if (mode != SYS_SICD)
		exynos_ss_pmu(mode, __func__, ESS_FLAG_OUT);

	return 0;
}

/*
 * crownlte suspend S0 (Documentation/crownlte/suspend-s0.md). In the enter
 * sequence, the INTR_GEN GRP2_INTR_BID_ENABLE write (0x14070200, CPU0 bit)
 * is what lets the CPU0 "core down" event reach the APM firmware, which
 * then runs the SoC/MIF sleep sequencers. The other two writes only clear
 * GRP1 and let the PMU latch the event (PMU_EVENT_INTERRUPT_ENABLE_GRP2);
 * a latched event with the INTR_GEN enable clear is not delivered (the same
 * holds for every core at runtime, see cpuidle-exynos9810.c).
 */
#define PMUCAL_INTR_GEN_PA		0x14070000
#define PMUCAL_GRP2_INTR_BID_ENABLE	0x0200

static bool pmucal_seq_arms_grp2(const struct pmucal_seq *s)
{
	return s->base_pa == PMUCAL_INTR_GEN_PA &&
	       s->offset == PMUCAL_GRP2_INTR_BID_ENABLE &&
	       (s->access_type == PMUCAL_WRITE ||
		s->access_type == PMUCAL_COND_WRITE ||
		s->access_type == PMUCAL_WRITE_WAIT ||
		s->access_type == PMUCAL_WRITE_RETURN) &&
	       (s->value & s->mask);
}

/**
 *  pmucal_system_enter_unarmed - pmucal_system_enter() without the write
 *		that hands the CPU0 core-down event to the APM firmware.
 *
 *  @mode: index of system power mode described in pmucal_system.h.
 *  @skipped: number of enter-sequence writes left out (GRP2 arming).
 *
 *  Same CPU_INFORM hint, register save and enter sequence as
 *  pmucal_system_enter(), except every write that sets an enable bit in
 *  INTR_GEN GRP2_INTR_BID_ENABLE. Refuses (-ENOENT, nothing written) when
 *  the enter sequence has no such write, so a table change cannot turn this
 *  into a silently armed entry. pmucal_system_exit()/_earlywakeup() undo it
 *  exactly as they undo pmucal_system_enter().
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int pmucal_system_enter_unarmed(int mode, unsigned int *skipped)
{
	struct pmucal_seq *seq;
	unsigned int i, n, arms = 0;
	int ret;

	*skipped = 0;
	if (mode < 0 || mode >= NUM_SYS_POWERDOWN || !pmucal_lpm_list[mode].enter)
		return -EINVAL;

	seq = pmucal_lpm_list[mode].enter;
	n = pmucal_lpm_list[mode].num_enter;
	for (i = 0; i < n; i++)
		if (pmucal_seq_arms_grp2(&seq[i]))
			arms++;
	if (!arms) {
		pr_err("%s %s: mode %d enter sequence has no GRP2 arming write; refused\n",
		       PMUCAL_PREFIX, __func__, mode);
		return -ENOENT;
	}

	pmucal_powermode_hint(pmucal_sys_powermode[mode]);
	pmucal_rae_save_seq(pmucal_lpm_list[mode].save,
			    pmucal_lpm_list[mode].num_save);

	for (i = 0; i < n; i++) {
		if (pmucal_seq_arms_grp2(&seq[i])) {
			(*skipped)++;
			continue;
		}
		ret = pmucal_rae_handle_seq(&seq[i], 1);
		if (ret) {
			pr_err("%s %s: error on handling enter sequence. (mode : %d)\n",
			       PMUCAL_PREFIX, __func__, mode);
			return ret;
		}
	}

	return 0;
}

/**
 *  pmucal_system_exit - prepares to exit a system power mode.
 *		        exposed to PWRCAL interface.
 *
 *  @mode: index of system power mode described in pmucal_system.h.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int pmucal_system_exit(int mode)
{
	int ret;

	if (mode != SYS_SICD)
		exynos_ss_pmu(mode, __func__, ESS_FLAG_IN);

	if (mode >= NUM_SYS_POWERDOWN) {
		pr_err("%s %s: mode index(%d) is out of supported range (0~%d).\n",
				PMUCAL_PREFIX, __func__, mode, NUM_SYS_POWERDOWN);
		return -EINVAL;
	}

	if (!pmucal_lpm_list[mode].exit) {
		pr_err("%s %s: there is no sequence element for exiting mode(%d).\n",
				PMUCAL_PREFIX, __func__, mode);
		return -ENOENT;
	}

	ret = pmucal_rae_handle_seq(pmucal_lpm_list[mode].exit,
				pmucal_lpm_list[mode].num_exit);
	if (ret) {
		pr_err("%s %s: error on handling exit sequence. (mode : %d)\n",
				PMUCAL_PREFIX, __func__, mode);
		return ret;
	}

	ret = pmucal_rae_restore_seq(pmucal_lpm_list[mode].save,
				pmucal_lpm_list[mode].num_save);
	if (ret) {
		pr_err("%s %s: error on handling restore sequence. (mode : %d)\n",
				PMUCAL_PREFIX, __func__, mode);
		return ret;
	}

	if (mode != SYS_SICD)
		exynos_ss_pmu(mode, __func__, ESS_FLAG_OUT);

	return 0;
}

/**
 *  pmucal_system_earlywakeup - prepares to early-wakeup from a system power mode.
 *				exposed to PWRCAL interface.
 *
 *  @mode: index of system power mode described in pmucal_system.h.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int pmucal_system_earlywakeup(int mode)
{
	int ret;

	if (mode != SYS_SICD)
		exynos_ss_pmu(mode, __func__, ESS_FLAG_IN);

	if (mode >= NUM_SYS_POWERDOWN) {
		pr_err("%s %s: mode index(%d) is out of supported range (0~%d).\n",
				PMUCAL_PREFIX, __func__, mode, NUM_SYS_POWERDOWN);
		return -EINVAL;
	}

	if (!pmucal_lpm_list[mode].early_wakeup) {
		pr_err("%s %s: there is no sequence element for early_wkup mode(%d).\n",
				PMUCAL_PREFIX, __func__, mode);
		return -ENOENT;
	}

	ret = pmucal_rae_handle_seq(pmucal_lpm_list[mode].early_wakeup,
				pmucal_lpm_list[mode].num_early_wakeup);
	if (ret) {
		pr_err("%s %s: error on handling elry_wkup sequence. (mode : %d)\n",
				PMUCAL_PREFIX, __func__, mode);
		return ret;
	}

	ret = pmucal_rae_restore_seq(pmucal_lpm_list[mode].save,
				pmucal_lpm_list[mode].num_save);
	if (ret) {
		pr_err("%s %s: error on handling restore sequence. (mode : %d)\n",
				PMUCAL_PREFIX, __func__, mode);
		return ret;
	}
	pmucal_powermode_hint_clear();

	if (mode != SYS_SICD)
		exynos_ss_pmu(mode, __func__, ESS_FLAG_OUT);

	return 0;
}

/**
 *  pmucal_system_init - Init function of PMUCAL SYSTEM common logic.
 *		            exposed to PWRCAL interface.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int __init pmucal_system_init(void)
{
	int ret = 0, i;

	if (!pmucal_lpm_init_size || !pmucal_lpm_list_size) {
		pr_err("%s %s: there is no lpm init seq or lpm list. aborting init...\n",
				PMUCAL_PREFIX, __func__);
		return -ENOENT;
	}

	/* convert physical base address to virtual addr */
	for (i = 0; i < NUM_SYS_POWERDOWN; i++) {
		/* skip non-existing power mode */
		if (!pmucal_lpm_list[i].num_enter && !pmucal_lpm_list[i].num_exit && !pmucal_lpm_list[i].num_early_wakeup)
			continue;

		ret = pmucal_rae_phy2virt(pmucal_lpm_list[i].enter,
					pmucal_lpm_list[i].num_enter);
		if (ret) {
			pr_err("%s %s: error on PA2VA conversion. seq:enter, mode_id:%d. aborting init...\n",
					PMUCAL_PREFIX, __func__, i);
			goto out;
		}

		ret = pmucal_rae_phy2virt(pmucal_lpm_list[i].save,
					pmucal_lpm_list[i].num_save);
		if (ret) {
			pr_err("%s %s: error on PA2VA conversion. seq:save, mode_id:%d. aborting init...\n",
					PMUCAL_PREFIX, __func__, i);
			goto out;
		}

		ret = pmucal_rae_phy2virt(pmucal_lpm_list[i].exit,
					pmucal_lpm_list[i].num_exit);
		if (ret) {
			pr_err("%s %s: error on PA2VA conversion. seq:exit, mode_id:%d. aborting init...\n",
					PMUCAL_PREFIX, __func__, i);
			goto out;
		}

		ret = pmucal_rae_phy2virt(pmucal_lpm_list[i].early_wakeup,
					pmucal_lpm_list[i].num_early_wakeup);
		if (ret) {
			pr_err("%s %s: error on PA2VA conversion. seq:erlywkup, mode_id:%d. aborting init...\n",
					PMUCAL_PREFIX, __func__, i);
			goto out;
		}
	}

	ret = pmucal_rae_phy2virt(pmucal_lpm_init, pmucal_lpm_init_size);
	if (ret) {
		pr_err("%s %s: error on PA2VA conversion for lpm_init seq. aborting init...\n",
				PMUCAL_PREFIX, __func__);
		goto out;
	}

	/* do default settings for PMU */
	ret = pmucal_rae_handle_seq(pmucal_lpm_init, pmucal_lpm_init_size);
	if (ret) {
		pr_err("%s %s: error on handling lpm_init sequence.\n",
				PMUCAL_PREFIX, __func__);
		goto out;
	}

out:
	return ret;
}
