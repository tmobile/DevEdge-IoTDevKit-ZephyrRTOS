/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <cmsis_os.h>

#define ONESHOT_TIME	1000
#define PERIOD		500
#define NUM_PERIODS	5

void Timer1_Callback(void const *arg);
void Timer2_Callback(void const *arg);

osTimerDef(Timer1, Timer1_Callback);
osTimerDef(Timer2, Timer2_Callback);

uint32_t num_oneshots_executed;
uint32_t num_periods_executed;

void Timer1_Callback(void const *arg)
{
	uint32_t Tmr = *(uint32_t *)arg;

	num_oneshots_executed++;
	TC_PRINT("oneshot_callback (Timer %d) = %d\n",
		 Tmr, num_oneshots_executed);
}

void Timer2_Callback(void const *arg)
{
	uint32_t Tmr = *(uint32_t *)arg;

	num_periods_executed++;
	TC_PRINT("periodic_callback (Timer %d) = %d\n",
		 Tmr, num_periods_executed);
}

ZTEST(cmsis_timer, test_timer)
{
	osTimerId id1;
	osTimerId id2;
	uint32_t  exec1;
	uint32_t  exec2;
	osStatus status;
	uint32_t timerDelay;

	/* Create one-shot timer */
	exec1 = 1U;
	id1 = osTimerCreate(osTimer(Timer1), osTimerOnce, &exec1);
	zassert_true(id1 != NULL, "error creating one-shot timer");

	/* Stop the timer before start */
	status = osTimerStop(id1);
	zassert_true(status == osErrorResource, "error while stopping non-active timer");

	timerDelay = ONESHOT_TIME;
	status = osTimerStart(id1, timerDelay);
	zassert_true(status == osOK, "error starting one-shot timer");

	/* Timer should fire only once if setup in one shot
	 * mode. Wait for 3 times the one-shot time to see
	 * if it fires more than once.
	 */
	osDelay(timerDelay*3U + 100);
	zassert_true(num_oneshots_executed == 1U,
			"error setting up one-shot timer");

	status = osTimerStop(id1);
	zassert_true(status == osOK, "error stopping one-shot timer");

	status = osTimerDelete(id1);
	zassert_true(status == osOK, "error deleting one-shot timer");

	/* Create periodic timer */
	exec2 = 2U;
	id2 = osTimerCreate(osTimer(Timer2), osTimerPeriodic, &exec2);
	zassert_true(id2 != NULL, "error creating periodic timer");

	timerDelay = PERIOD;
	status = osTimerStart(id2, timerDelay);
	zassert_true(status == osOK, "error starting periodic timer");

	/* Timer should fire periodically if setup in periodic
	 * mode. Wait for NUM_PERIODS periods to see if it is
	 * fired NUM_PERIODS times.
	 */
	osDelay(timerDelay*NUM_PERIODS + 100);

	zassert_true(num_periods_executed == NUM_PERIODS,
			"error setting up periodic timer");

	/* Delete the timer before stop */
	status = osTimerDelete(id2);
	zassert_true(status == osOK, "error deleting periodic timer");
}
ZTEST_SUITE(cmsis_timer, NULL, NULL, NULL, NULL, NULL);
