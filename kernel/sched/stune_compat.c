// SPDX-License-Identifier: GPL-2.0
/* Private crownlte legacy ABI adapter. Not the removed Samsung EMS scheduler. */
#include <linux/cgroup.h>
#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <trace/hooks/sched.h>
#include "sched.h"
#include "stune_compat.h"

struct stune_css {
	struct cgroup_subsys_state css;
	int idx, boost, prefer_perf;
	bool prefer_idle;
};
struct stune_cpu {
	raw_spinlock_t lock;
	unsigned int count[STUNE_GROUPS];
};
static DEFINE_PER_CPU(struct stune_cpu, stune_cpu_state);
static DEFINE_MUTEX(stune_groups_lock);
static bool used[STUNE_GROUPS] = { true };
static int group_boost[STUNE_GROUPS];
static struct stune_css root_stune;
static bool stune_ready;

static struct stune_css *css_stune(struct cgroup_subsys_state *css)
{
	return container_of(css, struct stune_css, css);
}

/* Caller holds rq and RCU as appropriate. The CSS is the real controller CSS. */
static struct stune_css *task_stune(struct task_struct *p)
{
	return css_stune(task_css(p, schedtune_cgrp_id));
}

static void stune_enqueue_idx(struct rq *rq, struct task_struct *p, int idx)
{
	struct stune_cpu *sc = &per_cpu(stune_cpu_state, cpu_of(rq));
	unsigned long flags;
	lockdep_assert_rq_held(rq);
	if (WARN_ON_ONCE(p->schedtune_accounted && p->schedtune_cpu != cpu_of(rq)))
		return;
	raw_spin_lock_irqsave(&sc->lock, flags);
	if (!WARN_ON_ONCE(!stune_count_enqueue(sc->count, &p->schedtune_idx,
			&p->schedtune_accounted, idx)))
		p->schedtune_cpu = cpu_of(rq);
	raw_spin_unlock_irqrestore(&sc->lock, flags);
}

static void stune_enqueue(void *unused, struct rq *rq, struct task_struct *p, int flags)
{
	if (!READ_ONCE(stune_ready))
		return;
	/* EEVDF delayed entities remain queued but are not runnable. */
	if ((p->sched_class == &fair_sched_class && p->se.sched_delayed) ||
	    (p->flags & PF_EXITING))
		return;
	rcu_read_lock();
	stune_enqueue_idx(rq, p, task_stune(p)->idx);
	rcu_read_unlock();
}

static void stune_dequeue(void *unused, struct rq *rq, struct task_struct *p, int flags)
{
	struct stune_cpu *sc = &per_cpu(stune_cpu_state, cpu_of(rq));
	unsigned long irqflags;
	if (!READ_ONCE(stune_ready))
		return;
	lockdep_assert_rq_held(rq);
	if (WARN_ON_ONCE(p->schedtune_accounted && p->schedtune_cpu != cpu_of(rq)))
		return;
	raw_spin_lock_irqsave(&sc->lock, irqflags);
	WARN_ON_ONCE(!stune_count_dequeue(sc->count, p->schedtune_idx,
					 &p->schedtune_accounted));
	raw_spin_unlock_irqrestore(&sc->lock, irqflags);
}

static void stune_fork(struct task_struct *p)
{
	p->schedtune_idx = 0;
	p->schedtune_cpu = -1;
	p->schedtune_accounted = false;
}

static void stune_exit(struct task_struct *p)
{
	struct rq_flags rf;
	struct rq *rq = task_rq_lock(p, &rf);
	stune_dequeue(NULL, rq, p, 0);
	task_rq_unlock(rq, p, &rf);
}

static void stune_attach(struct cgroup_taskset *tset)
{
	struct task_struct *p;
	struct cgroup_subsys_state *css;
	cgroup_taskset_for_each(p, css, tset) {
		struct rq_flags rf;
		struct rq *rq = task_rq_lock(p, &rf);
		/* Account after the committed CSS switch, never in can_attach. */
		if (!(p->flags & PF_EXITING) && (p->schedtune_accounted ||
		    (task_on_rq_queued(p) &&
		     (p->sched_class != &fair_sched_class || !p->se.sched_delayed))))
			stune_enqueue_idx(rq, p, css_stune(css)->idx);
		task_rq_unlock(rq, p, &rf);
	}
}

static struct cgroup_subsys_state *stune_alloc(struct cgroup_subsys_state *parent)
{
	struct stune_css *st;
	int idx, cpu;
	if (!parent) {
		for_each_possible_cpu(cpu)
			raw_spin_lock_init(&per_cpu(stune_cpu_state, cpu).lock);
		return &root_stune.css;
	}
	if (!READ_ONCE(stune_ready))
		return ERR_PTR(-ENODEV);
	if (parent != &root_stune.css)
		return ERR_PTR(-ENOMEM);
	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return ERR_PTR(-ENOMEM);
	mutex_lock(&stune_groups_lock);
	idx = stune_free_slot(used);
	if (idx < 0) {
		mutex_unlock(&stune_groups_lock);
		kfree(st);
		return ERR_PTR(-ENOSPC);
	}
	st->idx = idx;
	WRITE_ONCE(group_boost[idx], 0);
	WRITE_ONCE(used[idx], true);
	mutex_unlock(&stune_groups_lock);
	return &st->css;
}

static void stune_free(struct cgroup_subsys_state *css)
{
	struct stune_css *st = css_stune(css);
	int cpu;
	bool busy = false;
	if (css == &root_stune.css)
		return;
	mutex_lock(&stune_groups_lock);
	for_each_possible_cpu(cpu) {
		struct stune_cpu *sc = &per_cpu(stune_cpu_state, cpu);
		unsigned long flags;
		raw_spin_lock_irqsave(&sc->lock, flags);
		busy |= !!sc->count[st->idx];
		raw_spin_unlock_irqrestore(&sc->lock, flags);
	}
	WRITE_ONCE(group_boost[st->idx], 0);
	/* A broken lifetime invariant must not alias an old task to a reused slot. */
	if (!WARN_ON_ONCE(busy))
		WRITE_ONCE(used[st->idx], false);
	mutex_unlock(&stune_groups_lock);
	kfree(st);
}

static s64 stune_boost_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	return READ_ONCE(css_stune(css)->boost);
}

static int stune_boost_write(struct cgroup_subsys_state *css, struct cftype *cft, s64 value)
{
	struct stune_css *st = css_stune(css);
	if (!READ_ONCE(stune_ready))
		return -ENODEV;
	if (value < -100 || value > 100)
		return -EINVAL;
	WRITE_ONCE(st->boost, value);
	WRITE_ONCE(group_boost[st->idx], value);
	return 0;
}

static u64 stune_flag_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct stune_css *st = css_stune(css);
	return cft->private ? READ_ONCE(st->prefer_perf) : READ_ONCE(st->prefer_idle);
}

static int stune_flag_write(struct cgroup_subsys_state *css, struct cftype *cft, u64 value)
{
	struct stune_css *st = css_stune(css);
	if (!READ_ONCE(stune_ready))
		return -ENODEV;
	if (cft->private)
		WRITE_ONCE(st->prefer_perf, (int)value);
	else
		WRITE_ONCE(st->prefer_idle, !!value);
	return 0;
}

static struct cftype stune_files[] = {
	{ .name = "boost", .read_s64 = stune_boost_read, .write_s64 = stune_boost_write },
	{ .name = "prefer_idle", .read_u64 = stune_flag_read, .write_u64 = stune_flag_write },
	{ .name = "prefer_perf", .private = 1, .read_u64 = stune_flag_read, .write_u64 = stune_flag_write },
	{ }
};

struct cgroup_subsys schedtune_cgrp_subsys = {
	.css_alloc = stune_alloc, .css_free = stune_free,
	.attach = stune_attach, .fork = stune_fork, .exit = stune_exit,
	.legacy_cftypes = stune_files,
};

static void stune_cpu_util(void *unused, int cpu, unsigned long *util)
{
	struct stune_cpu *sc = &per_cpu(stune_cpu_state, cpu);
	unsigned long flags, sample;
	int i, boost[STUNE_GROUPS], result;
	bool valid[STUNE_GROUPS];
	if (!READ_ONCE(stune_ready))
		return;
	raw_spin_lock_irqsave(&sc->lock, flags);
	for (i = 0; i < STUNE_GROUPS; i++) {
		boost[i] = READ_ONCE(group_boost[i]);
		valid[i] = READ_ONCE(used[i]);
	}
	result = stune_active_boost(sc->count, boost, valid);
	raw_spin_unlock_irqrestore(&sc->lock, flags);
	/* Zero boost retains ACK's normal runnable-util uplift and unset sentinel. */
	if (!result)
		return;
	sample = cpu_util_cfs(cpu);
	/* Preserve the modern runnable-average floor as well as restored SPC. */
	sample = max(sample, READ_ONCE(cpu_rq(cpu)->cfs.avg.runnable_avg));
	*util = stune_spc(arch_scale_cpu_capacity(cpu), sample, result);
}

static void stune_select_cpu(void *unused, struct task_struct *p, int prev_cpu,
			    int sd_flag, int wake_flags, int *new_cpu)
{
	struct stune_candidate best = { .cpu = -1 };
	struct stune_css *st;
	unsigned long task_util, unclamped;
	int cpu, boost, perf;
	bool prefer_idle;
	if (!READ_ONCE(stune_ready) || *new_cpu >= 0 || !(wake_flags & WF_TTWU) ||
	    (sd_flag & SD_BALANCE_FORK))
		return;
	rcu_read_lock();
	st = task_stune(p);
	boost = READ_ONCE(st->boost);
	perf = READ_ONCE(st->prefer_perf);
	prefer_idle = READ_ONCE(st->prefer_idle);
	if (!prefer_idle && perf <= 0)
		goto out;
	/* Mainline PELT replaces the old Samsung SSE/USS private utilization. */
	task_util = READ_ONCE(p->se.avg.util_avg);
	for_each_cpu_and(cpu, p->cpus_ptr, cpu_active_mask) {
		struct cpuidle_state *idle_state = idle_get_state(cpu_rq(cpu));
		unsigned long capacity = READ_ONCE(cpu_rq(cpu)->cpu_capacity);
		unsigned long base = cpu_util_cfs(cpu);
		struct stune_candidate candidate;
		unclamped = stune_spc(arch_scale_cpu_capacity(cpu), task_util, boost);
		unclamped = clamp(unclamped, uclamp_eff_value(p, UCLAMP_MIN),
					    uclamp_eff_value(p, UCLAMP_MAX));
		/* Avoid counting the waking task's residual contribution twice. */
		if (cpu == task_cpu(p))
			base -= min(base, task_util);
		candidate.cpu = cpu;
		candidate.capacity = capacity;
		candidate.util = base + unclamped;
		candidate.idle = idle_cpu(cpu);
		candidate.latency = idle_state ? idle_state->exit_latency_ns : 0;
		if (stune_candidate_better(&candidate, &best, perf > 0, boost))
			best = candidate;
	}
	if (best.cpu >= 0 && cpu_active(best.cpu) && cpumask_test_cpu(best.cpu, p->cpus_ptr))
		*new_cpu = best.cpu;
out:
	rcu_read_unlock();
}

static int __init stune_init(void)
{
	int ret;
	ret = register_trace_android_rvh_after_enqueue_task(stune_enqueue, NULL);
	if (ret)
		goto failed;
	ret = register_trace_android_rvh_dequeue_task(stune_dequeue, NULL);
	if (ret)
		goto failed;
	ret = register_trace_android_rvh_cpu_util_cfs_boost(stune_cpu_util, NULL);
	if (ret)
		goto failed;
	ret = register_trace_android_rvh_select_task_rq_fair(stune_select_cpu, NULL);
	if (ret)
		goto failed;
	WRITE_ONCE(stune_ready, true);
	pr_info("schedtune: real legacy groups with SPC and wake-up preference adapter\n");
	return 0;
failed:
	/* Restricted hooks are permanent. Installed callbacks remain inert. */
	pr_err("schedtune: scheduler hook registration failed: %d; controller writes disabled\n", ret);
	return ret;
}
core_initcall(stune_init);
