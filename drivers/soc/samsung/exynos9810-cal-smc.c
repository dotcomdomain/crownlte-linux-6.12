// SPDX-License-Identifier: GPL-2.0
#include <linux/arm-smccc.h>
#include <linux/export.h>
#include <linux/smc.h>
#include <asm/barrier.h>

int __exynos_smc(unsigned long cmd, unsigned long a1,
		 unsigned long a2, unsigned long a3)
{
	struct arm_smccc_res res;
	dsb(sy);
	arm_smccc_smc(cmd, a1, a2, a3, 0, 0, 0, 0, &res);
	return (int)res.a0;
}
int exynos_smc(unsigned long cmd, unsigned long a1,
	       unsigned long a2, unsigned long a3)
{
	return __exynos_smc(cmd, a1, a2, a3);
}
EXPORT_SYMBOL_GPL(exynos_smc);
int exynos_smc_readsfr(unsigned long addr, unsigned long *value)
{
	struct arm_smccc_res res;
	dsb(sy);
	/* The original assembly MOV W0 zero-extended this negative command. */
	arm_smccc_smc((u32)SMC_CMD_REG, (u32)SMC_REG_ID_SFR_R(addr),
		      0, 0, 0, 0, 0, 0, &res);
	if (!res.a0)
		*value = res.a2;
	return (int)res.a0;
}
int exynos_pd_tz_save(unsigned int addr)
{
	return exynos_smc(SMC_CMD_PREAPRE_PD_ONOFF, 0, addr, 0);
}
int exynos_pd_tz_restore(unsigned int addr)
{
	return exynos_smc(SMC_CMD_PREAPRE_PD_ONOFF, 1, addr, 0);
}
