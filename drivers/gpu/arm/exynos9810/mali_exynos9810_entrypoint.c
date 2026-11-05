// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos9810 (crownlte): the kbase core hooks of the Samsung r38p1 tree
 * (mali_exynos_kbase_entrypoint.h) and the Samsung private ioctls.
 *
 * The hooks feed Samsung's gpex profiling/boost/LLC/secure-rendering stacks
 * on 4.9; none of those exist here, so they are no-ops with the same return
 * values as the 4.9 "dummy" backends (protected mode: -ENOSYS).
 *
 * Ioctl semantics are those of 4.9 mali_exynos_ioctl(), which the kbase core
 * calls for every command it does not handle itself:
 *  - a known Samsung command copies its argument (-EFAULT on a bad pointer)
 *    and returns its handler's value; a non-zero value makes the core log
 *    "Unknown ioctl" and return -ENOIOCTLCMD (userspace sees ENOTTY);
 *  - any other command returns 0 (4.9 returned 0 from the switch default).
 * The vendor libGLES_mali.so issues MALI_EXYNOS_IOCTL_EHMP (0x4004802a) with
 * EHMP_SET/EHMP_UNSET; 4.9 then boosted the big cluster (gpexwa_ehmp); here
 * it is accepted and ignored.
 */
#include <linux/dma-buf.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <mali_kbase.h>
#include <mali_exynos_ioctl.h>

#include "mali_exynos_kbase_entrypoint.h"
#include "mali_exynos9810.h"

void mali_exynos_update_job_load(struct kbase_jd_atom *katom, ktime_t *end_timestamp) { }
int mali_exynos_set_pm_state_resume_begin(void) { return 0; }
int mali_exynos_set_pm_state_resume_end(void) { return 0; }

/* No LLC / IO coherency on Exynos9810 (CONFIG_MALI_EXYNOS_LLC=n in 4.9). */
void mali_exynos_coherency_reg_map(void) { }
void mali_exynos_coherency_reg_unmap(void) { }
void mali_exynos_coherency_set_coherency_feature(void) { }
void mali_exynos_llc_set_aruser(void) { }
void mali_exynos_llc_set_awuser(void) { }

void mali_exynos_update_jobslot_util(int slot, bool gpu_active, u32 ns_time) { }
void mali_exynos_set_jobslot_status(int slot, bool is_active) { }
int mali_exynos_set_count(struct kbase_jd_atom *katom, u32 status, bool stop) { return 0; }

void mali_exynos_update_firstjob_vsync_time(void) { }
void mali_exynos_update_firstjob_time(void) { }
void mali_exynos_update_lastjob_time(int slot_nr) { }
void mali_exynos_update_jobsubmit_time(void) { }
void mali_exynos_sum_jobs_time(int slot_nr) { }
void mali_exynos_amigo_interframe_hw_update_eof(void) { }
void mali_exynos_amigo_interframe_hw_update(void) { }

int mali_exynos_get_gpu_power_state(void)
{
	struct exynos9810_gpu *g = exynos9810_gpu;

	return g && xg_pd_is_on(g, NULL, NULL) ? 1 : 0;
}

void mali_exynos_set_thread_priority(struct kbase_context *kctx) { }
void mali_exynos_set_thread_affinity(void) { }

/* Secure rendering not configured (4.9 gpexbe_secure_dummy.c). */
int mali_exynos_legacy_jm_enter_protected_mode(struct kbase_device *kbdev) { return -ENOSYS; }
int mali_exynos_legacy_jm_exit_protected_mode(struct kbase_device *kbdev) { return -ENOSYS; }
int mali_exynos_legacy_pm_exit_protected_mode(struct kbase_device *kbdev) { return -ENOSYS; }
struct protected_mode_ops *mali_exynos_get_protected_ops(void) { return NULL; }

/*
 * 4.9 asked ION whether the buffer is cached and skipped CPU cache
 * maintenance otherwise. With dma-buf heaps the uncached heaps export as
 * "<heap>-uncached"; everything else is synced (always correct).
 */
bool mali_exynos_dmabuf_is_cached(struct dma_buf *dmabuf)
{
	const char *name = dmabuf ? dmabuf->exp_name : NULL;
	size_t len;

	if (!name)
		return true;
	len = strlen(name);
	return !(len >= 9 && !strcmp(name + len - 9, "-uncached"));
}

void mali_exynos_sysfs_set_gpu_model_callback(sysfs_read_func show_gpu_model_fn) { }
void mali_exynos_debug_print_info(struct kbase_device *kbdev) { }

#define XG_IOCTL_IN(type)							\
	do {									\
		type param;							\
		if (copy_from_user(&param, uarg, sizeof(param)))		\
			return -EFAULT;						\
		return xg_##type(kctx, &param);					\
	} while (0)

#define XG_IOCTL_OUT(type)							\
	do {									\
		type param;							\
		memset(&param, 0, sizeof(param));				\
		if (copy_to_user(uarg, &param, sizeof(param)))			\
			return -EFAULT;						\
		return 0;							\
	} while (0)

typedef struct mali_exynos_ioctl_hcm_pmqos xg_hcm;
typedef struct mali_exynos_ioctl_cmar_boost xg_cmar;
typedef struct kbase_ioctl_slsi_singlebuffer_boost_flags xg_sbb;
typedef struct kbase_ioctl_slsi_egp xg_egp;
typedef struct mali_exynos_ioctl_interactive_boost xg_ib;
typedef struct mali_exynos_ioctl_ehmp_flags xg_ehmp;

static int xg_xg_hcm(struct kbase_context *kctx, xg_hcm *p) { return 0; }
static int xg_xg_cmar(struct kbase_context *kctx, xg_cmar *p) { return 0; }
static int xg_xg_egp(struct kbase_context *kctx, xg_egp *p) { return 0; }
static int xg_xg_ib(struct kbase_context *kctx, xg_ib *p) { return 0; }
static int xg_xg_ehmp(struct kbase_context *kctx, xg_ehmp *p) { return 0; }

/* 4.9 kbase_api_singlebuffer_boost(): only the two legacy UK ids are valid. */
static int xg_xg_sbb(struct kbase_context *kctx, xg_sbb *p)
{
	return (p->flags == 561 || p->flags == 562) ? 0 : -EINVAL;
}

int mali_exynos_ioctl(struct kbase_context *kctx, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;

	switch (cmd) {
	case MALI_EXYNOS_IOCTL_AMIGO_FLAGS:
		XG_IOCTL_OUT(struct mali_exynos_ioctl_amigo_flags);
	case MALI_EXYNOS_IOCTL_READ_GTS_INFO:
		XG_IOCTL_OUT(struct mali_exynos_ioctl_gts_info);
	case MALI_EXYNOS_IOCTL_HCM_PMQOS:
		XG_IOCTL_IN(xg_hcm);
	case MALI_EXYNOS_IOCTL_MEM_USAGE_ADD:
		/* 4.9: deprecated, no copy, success */
		return 0;
	case MALI_EXYNOS_IOCTL_CMAR_BOOST:
		XG_IOCTL_IN(xg_cmar);
	case KBASE_IOCTL_SLSI_SINGLEBUFFER_BOOST_FLAGS:
		XG_IOCTL_IN(xg_sbb);
	case KBASE_IOCTL_SLSI_EGP:
		XG_IOCTL_IN(xg_egp);
	case MALI_EXYNOS_IOCTL_INTERACTIVE_BOOST:
		XG_IOCTL_IN(xg_ib);
	case MALI_EXYNOS_IOCTL_EHMP:
		XG_IOCTL_IN(xg_ehmp);
	}
	return 0;
}
