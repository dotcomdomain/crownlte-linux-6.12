/*
 * Copyright (c) 2013-2015 TRUSTONIC LIMITED
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef _MC_LOGGING_H_
#define _MC_LOGGING_H_

void mc_logging_run(void);
int  mc_logging_init(void);
void mc_logging_exit(void);
int mc_logging_start(void);
void mc_logging_stop(void);

/* crownlte boot self-test helper: read-only, no SMC */
struct mc_logging_selftest_info {
	phys_addr_t	pa;
	size_t		size;
	bool		shared;
	u32		version;
	u32		length;
	u32		head;
};

void mc_logging_get_selftest_info(struct mc_logging_selftest_info *info);

#endif /* _MC_LOGGING_H_ */
