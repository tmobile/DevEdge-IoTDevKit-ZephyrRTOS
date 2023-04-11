/*
 * Copyright 2021 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include "common.h"

static void test_step_1(void)
{
}

static bool predicate(const void *state)
{
	const struct global_test_state *s = state;

	return s->phase == PHASE_STEPS_1;
}

ztest_register_test_suite(test_step_1, predicate, ztest_unit_test(test_step_1));
