/*
 * Copyright (c) 2018, Piotr Mienkowski
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <em_emu.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

/*
 * Power state map:
 * PM_STATE_RUNTIME_IDLE: EM1 Sleep
 * PM_STATE_SUSPEND_TO_IDLE: EM2 Deep Sleep
 * PM_STATE_STANDBY: EM3 Stop
 */

/* Invoke Low Power/System Off specific Tasks */
__weak void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	LOG_DBG("SoC entering power state %d", state);

	/* FIXME: When this function is entered the Kernel has disabled
	 * interrupts using BASEPRI register. This is incorrect as it prevents
	 * waking up from any interrupt which priority is not 0. Work around the
	 * issue and disable interrupts using PRIMASK register as recommended
	 * by ARM.
	 */

	/* Set PRIMASK */
	__disable_irq();
	/* Set BASEPRI to 0 */
	irq_unlock(0);

	switch (state) {
	case PM_STATE_RUNTIME_IDLE:
		EMU_EnterEM1();
		break;
	case PM_STATE_SUSPEND_TO_IDLE:
		EMU_EnterEM2(true);
		/* Deep Sleep Flag is not properly cleared on Wake */
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		break;
	case PM_STATE_STANDBY:
		EMU_EnterEM3(true);
		/* Deep Sleep Flag is not properly cleared on Wake */
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		break;
	case PM_STATE_SOFT_OFF:
		{
		EMU_EM4Init_TypeDef em4_state = {
			.em4State = substate_id == 1 ? emuEM4Hibernate : emuEM4Shutoff,
			.pinRetentionMode = emuPinRetentionEm4Exit,
			.retainLfrco = (substate_id == 1),
			.retainLfxo = (substate_id == 1),
			.retainUlfrco = (substate_id == 1),
			.vScaleEM4HVoltage = emuVScaleEM4H_LowPower
		};
		EMU_EM4Init(&em4_state);
		EMU_EnterEM4();
		break;
		}
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}

	LOG_DBG("SoC leaving power state %d", state);

	/* Clear PRIMASK */
	__enable_irq();
}

/* Handle SOC specific activity after Low Power Mode Exit */
__weak void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);
}
