/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/toolchain.h>

#include <stm32_ll_cortex.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_system.h>

void z_sys_poweroff(void)
{
	LL_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);
	LL_LPM_EnableDeepSleep();
	LL_DBGMCU_DisableDBGStandbyMode();

	k_cpu_idle();

	CODE_UNREACHABLE;
}
