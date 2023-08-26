/* fifo_b.c */

/*
 * Copyright (c) 1997-2010, 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "master.h"

#ifdef FIFO_BENCH

/**
 *
 * @brief Queue transfer speed test
 *
 */
void queue_test(void)
{
	uint32_t et; /* elapsed time */
	int i;

	PRINT_STRING(dashline);
	et = BENCH_START();
	for (i = 0; i < NR_OF_FIFO_RUNS; i++) {
		k_msgq_put(&DEMOQX1, data_bench, K_FOREVER);
	}
	et = TIME_STAMP_DELTA_GET(et);

	PRINT_F(FORMAT, "enqueue 1 byte msg in FIFO",
		SYS_CLOCK_HW_CYCLES_TO_NS_AVG(et, NR_OF_FIFO_RUNS));

	et = BENCH_START();
	for (i = 0; i < NR_OF_FIFO_RUNS; i++) {
		k_msgq_get(&DEMOQX1, data_bench, K_FOREVER);
	}
	et = TIME_STAMP_DELTA_GET(et);
	check_result();

	PRINT_F(FORMAT, "dequeue 1 byte msg in FIFO",
		SYS_CLOCK_HW_CYCLES_TO_NS_AVG(et, NR_OF_FIFO_RUNS));

	et = BENCH_START();
	for (i = 0; i < NR_OF_FIFO_RUNS; i++) {
		k_msgq_put(&DEMOQX4, data_bench, K_FOREVER);
	}
	et = TIME_STAMP_DELTA_GET(et);
	check_result();

	PRINT_F(FORMAT, "enqueue 4 bytes msg in FIFO",
		SYS_CLOCK_HW_CYCLES_TO_NS_AVG(et, NR_OF_FIFO_RUNS));

	et = BENCH_START();
	for (i = 0; i < NR_OF_FIFO_RUNS; i++) {
		k_msgq_get(&DEMOQX4, data_bench, K_FOREVER);
	}
	et = TIME_STAMP_DELTA_GET(et);
	check_result();

	PRINT_F(FORMAT, "dequeue 4 bytes msg in FIFO",
		SYS_CLOCK_HW_CYCLES_TO_NS_AVG(et, NR_OF_FIFO_RUNS));

	k_sem_give(&STARTRCV);

	et = BENCH_START();
	for (i = 0; i < NR_OF_FIFO_RUNS; i++) {
		k_msgq_put(&DEMOQX1, data_bench, K_FOREVER);
	}
	et = TIME_STAMP_DELTA_GET(et);
	check_result();

	PRINT_F(FORMAT,
		"enqueue 1 byte msg in FIFO to a waiting higher priority task",
		SYS_CLOCK_HW_CYCLES_TO_NS_AVG(et, NR_OF_FIFO_RUNS));

	et = BENCH_START();
	for (i = 0; i < NR_OF_FIFO_RUNS; i++) {
		k_msgq_put(&DEMOQX4, data_bench, K_FOREVER);
	}
	et = TIME_STAMP_DELTA_GET(et);
	check_result();

	PRINT_F(FORMAT,
		"enqueue 4 bytes in FIFO to a waiting higher priority task",
		SYS_CLOCK_HW_CYCLES_TO_NS_AVG(et, NR_OF_FIFO_RUNS));
}

#endif /* FIFO_BENCH */
