/* mutex_b.c */

/*
 * Copyright (c) 1997-2010, 2013-2015 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "master.h"

#ifdef MUTEX_BENCH

/**
 *
 * @brief Mutex lock/unlock test
 *
 */
void mutex_test(void)
{
	uint32_t et; /* elapsed time */
	int i;

	PRINT_STRING(dashline);
	et = BENCH_START();
	for (i = 0; i < NR_OF_MUTEX_RUNS; i++) {
		k_mutex_lock(&DEMO_MUTEX, K_FOREVER);
		k_mutex_unlock(&DEMO_MUTEX);
	}
	et = TIME_STAMP_DELTA_GET(et);
	check_result();

	PRINT_F(FORMAT, "average lock and unlock mutex",
		SYS_CLOCK_HW_CYCLES_TO_NS_AVG(et, (2 * NR_OF_MUTEX_RUNS)));
}

#endif /* MUTEX_BENCH */
