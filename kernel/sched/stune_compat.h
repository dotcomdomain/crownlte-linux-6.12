/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _STUNE_COMPAT_POLICY_H
#define _STUNE_COMPAT_POLICY_H

#ifndef __KERNEL__
#include <stdbool.h>
#endif
#define STUNE_GROUPS 10

static inline unsigned long stune_spc(unsigned long cap, unsigned long util,
				      int boost)
{
	if (util > cap)
		util = cap;
	if (boost >= 0)
		return util + (cap - util) * boost / 100;
	return util - util * (-boost) / 100;
}

static inline int stune_free_slot(const bool used[STUNE_GROUPS])
{
	int i;
	for (i = 1; i < STUNE_GROUPS; i++)
		if (!used[i])
			return i;
	return -1;
}

static inline bool stune_count_enqueue(unsigned int counts[STUNE_GROUPS],
		int *idx, bool *accounted, int dest)
{
	if (dest < 0 || dest >= STUNE_GROUPS || *idx < 0 || *idx >= STUNE_GROUPS)
		return false;
	if (*accounted && *idx == dest)
		return true;
	if (*accounted) {
		if (!counts[*idx])
			return false;
		counts[*idx]--;
	}
	counts[dest]++;
	*idx = dest;
	*accounted = true;
	return true;
}

static inline bool stune_count_dequeue(unsigned int counts[STUNE_GROUPS],
		int idx, bool *accounted)
{
	if (!*accounted)
		return true;
	if (idx < 0 || idx >= STUNE_GROUPS || !counts[idx])
		return false;
	counts[idx]--;
	*accounted = false;
	return true;
}

static inline int stune_active_boost(const unsigned int counts[STUNE_GROUPS],
		const int boost[STUNE_GROUPS], const bool valid[STUNE_GROUPS])
{
	int result = boost[0] > 0 ? boost[0] : 0;
	int i;
	for (i = 1; i < STUNE_GROUPS; i++)
		if (valid[i] && counts[i] && boost[i] > result)
			result = boost[i];
	return result;
}

struct stune_candidate {
	int cpu;
	bool idle;
	unsigned long capacity, util;
	unsigned long long latency;
};

static inline bool stune_candidate_better(const struct stune_candidate *a,
		const struct stune_candidate *b, bool perf, int boost)
{
	if (a->util > a->capacity)
		return false;
	if (b->cpu < 0)
		return true;
	if (a->idle != b->idle)
		return a->idle;
	if (perf) {
		if (a->idle) {
			if (a->capacity != b->capacity)
				return a->capacity > b->capacity;
			return a->latency < b->latency;
		}
		return a->capacity - a->util > b->capacity - b->util;
	}
	if (a->idle && a->latency != b->latency)
		return a->latency < b->latency;
	if (!a->idle && boost <= 0 && a->capacity != b->capacity)
		return a->capacity < b->capacity;
	return a->util < b->util;
}
#endif
