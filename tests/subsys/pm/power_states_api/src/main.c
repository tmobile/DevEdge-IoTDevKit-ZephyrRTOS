/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/pm/pm.h>

/* Last state has not declared a minimum residency, so it should be
 * set the default 0 value
 */
static struct pm_state_info infos[] = {{PM_STATE_SUSPEND_TO_IDLE, 0, 10000, 100},
	       {PM_STATE_SUSPEND_TO_RAM, 0, 50000, 500}, {PM_STATE_STANDBY, 0, 0}};
static enum pm_state states[] = {PM_STATE_SUSPEND_TO_IDLE,
			PM_STATE_SUSPEND_TO_RAM, PM_STATE_STANDBY};
static enum pm_state wrong_states[] = {PM_STATE_SUSPEND_TO_DISK,
		PM_STATE_SUSPEND_TO_RAM, PM_STATE_SUSPEND_TO_RAM};

ZTEST(power_states_1cpu, test_power_states)
{
	enum pm_state dts_states[] =
		PM_STATE_LIST_FROM_DT_CPU(DT_NODELABEL(power_states));
	struct pm_state_info dts_infos[] =
		PM_STATE_INFO_LIST_FROM_DT_CPU(DT_NODELABEL(power_states));
	uint32_t dts_states_len =
		DT_NUM_CPU_POWER_STATES(DT_NODELABEL(power_states));

	zassert_true(ARRAY_SIZE(states) == dts_states_len,
		     "Invalid number of pm states");
	zassert_true(memcmp(infos, dts_infos, sizeof(dts_infos)) == 0,
		     "Invalid pm_state_info array");
	zassert_true(memcmp(states, dts_states, sizeof(dts_states)) == 0,
		     "Invalid pm-states array");

	zassert_false(memcmp(wrong_states, dts_states, sizeof(dts_states)) == 0,
		     "Invalid pm-states array");
}

ZTEST_SUITE(power_states_1cpu, NULL, NULL, ztest_simple_1cpu_before,
			ztest_simple_1cpu_after, NULL);
