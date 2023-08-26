/* mailbox_b.c */

/*
 * Copyright (c) 1997-2010, 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "master.h"

#ifdef MAILBOX_BENCH

static struct k_mbox_msg message;

#ifdef FLOAT
#define PRINT_HEADER()                                                       \
	(PRINT_STRING                                                        \
	   ("|   size(B) |       time/packet (usec)       |          MB/sec" \
	    "                |\n"))
#define PRINT_ONE_RESULT()                                                   \
	PRINT_F("|%11u|%32.3f|%32f|\n", putsize, puttime / 1000.0,           \
		(1000.0 * putsize) / SAFE_DIVISOR(puttime))

#define PRINT_OVERHEAD()                                                     \
	PRINT_F("| message overhead:  %10.3f     usec/packet               " \
		"                |\n", empty_msg_put_time / 1000.0)

#define PRINT_XFER_RATE()                                                     \
	double netto_transfer_rate;                                           \
	netto_transfer_rate = 1000.0 * \
		(putsize >> 1) / SAFE_DIVISOR(puttime - empty_msg_put_time);  \
	PRINT_F("| raw transfer rate:     %10.3f MB/sec (without"             \
		" overhead)                 |\n", netto_transfer_rate)

#else
#define PRINT_HEADER()                                                       \
	(PRINT_STRING                                                        \
	   ("|   size(B) |       time/packet (nsec)       |          KB/sec" \
	    "                |\n"))

#define PRINT_ONE_RESULT()                                                   \
	PRINT_F("|%11u|%32u|%32u|\n", putsize, puttime,	                     \
		(uint32_t)                                                   \
		(((uint64_t)putsize * 1000000U) / SAFE_DIVISOR(puttime)))

#define PRINT_OVERHEAD()                                                     \
	PRINT_F("| message overhead:  %10u     nsec/packet                 " \
	     "              |\n", empty_msg_put_time)

#define PRINT_XFER_RATE()                                                    \
	PRINT_F("| raw transfer rate:     %10u KB/sec (without"              \
		" overhead)                 |\n",                            \
		(uint32_t)((uint64_t)(putsize >> 1) * 1000000U /             \
			   SAFE_DIVISOR(puttime - empty_msg_put_time)))

#endif


/*
 * Function prototypes.
 */
void mailbox_put(uint32_t size, int count, uint32_t *time);

/*
 * Function declarations.
 */

/**
 *
 * @brief Mailbox transfer speed test
 *
 */
void mailbox_test(void)
{
	uint32_t putsize;
	uint32_t puttime;
	int putcount;
	unsigned int empty_msg_put_time;
	struct getinfo getinfo;

	PRINT_STRING(dashline);
	PRINT_STRING("|                "
		     "M A I L B O X   M E A S U R E M E N T S"
		     "                      |\n");
	PRINT_STRING(dashline);
	PRINT_STRING("| Send mailbox message to waiting high "
		     "priority task and wait                 |\n");
	PRINT_F("| repeat for %4d times and take the "
		"average                                  |\n",
		NR_OF_MBOX_RUNS);
	PRINT_STRING(dashline);
	PRINT_HEADER();
	PRINT_STRING(dashline);
	k_sem_reset(&SEM0);
	k_sem_give(&STARTRCV);

	putcount = NR_OF_MBOX_RUNS;

	putsize = 0U;
	mailbox_put(putsize, putcount, &puttime);
	/* waiting for ack */
	k_msgq_get(&MB_COMM, &getinfo, K_FOREVER);
	PRINT_ONE_RESULT();
	empty_msg_put_time = puttime;
	for (putsize = 8U; putsize <= MESSAGE_SIZE; putsize <<= 1) {
		mailbox_put(putsize, putcount, &puttime);
		/* waiting for ack */
		k_msgq_get(&MB_COMM, &getinfo, K_FOREVER);
		PRINT_ONE_RESULT();
	}
	PRINT_STRING(dashline);
	PRINT_OVERHEAD();
	PRINT_XFER_RATE();
}


/**
 *
 * @brief Write the number of data chunks into the mailbox
 *
 * @param size    The size of the data chunk.
 * @param count   Number of data chunks.
 * @param time    The total time.
 *
 */
void mailbox_put(uint32_t size, int count, uint32_t *time)
{
	int i;
	unsigned int t;

	message.rx_source_thread = K_ANY;
	message.tx_target_thread = K_ANY;

	/* first sync with the receiver */
	k_sem_give(&SEM0);
	t = BENCH_START();
	for (i = 0; i < count; i++) {
		k_mbox_put(&MAILB1, &message, K_FOREVER);
	}
	t = TIME_STAMP_DELTA_GET(t);
	*time = SYS_CLOCK_HW_CYCLES_TO_NS_AVG(t, count);
	check_result();
}

#endif /* MAILBOX_BENCH */
