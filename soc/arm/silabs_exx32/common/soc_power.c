/*
 * Copyright (c) 2021, Kalyan Kumar
 * Copyright (c) 2018, Piotr Mienkowski
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>

#include <em_emu.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

#ifdef CONFIG_SOC_GECKO_DEV_INIT

#include <zephyr/arch/arm/aarch32/irq.h>

#include <ksched.h>
#include <sl_power_manager.h>
#include <sl_sleeptimer.h>

/* Tick count (in lf ticks) last time the tick count was updated in RTOS. */
static uint32_t last_update_lftick;

/* Expected sleep lf ticks */
static uint32_t expected_sleep_lf_ticks;

/* Total lf ticks slept */
static uint32_t total_slept_lf_ticks;

/* sleep timer frequency */
static uint32_t sleeptimer_freq;

/* indicates scheduling is required or not */
static bool isContextSwitchRequired;

/* to compensate the os ticks during sleep time */
void sys_clock_announce(int32_t ticks);

struct pm_state_info pm_state_active = {PM_STATE_ACTIVE, 0, 0, 0};
#endif

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

#ifdef CONFIG_SOC_GECKO_DEV_INIT

	sl_power_manager_em_t energy_mode;
	uint32_t rtcc_prio;

	/* Save RTCC IRQ priority */
	rtcc_prio = NVIC_GetPriority(RTCC_IRQn);
	/*
	 * When this function is entered the Kernel has disabled handling interrupts
	 * with priority other than 0. The Interrupt for the timer used to wake up
	 * the cpu has priority equal to 1. Manually set this priority to 0 so that
	 * cpu could exit sleep state.
	 *
	 * Note on priority value: set priority to -1 because z_arm_irq_priority_set
	 * offsets the priorities by 1.
	 * This function call will effectively set the priority to 0.
	 */
	z_arm_irq_priority_set(RTCC_IRQn, -1, 0);

	switch (state) {

	case PM_STATE_STANDBY:
		energy_mode = SL_POWER_MANAGER_EM3;

		/* Limit sleep level to given state */
		sl_power_manager_add_em_requirement(energy_mode);

		sleeptimer_freq = sl_sleeptimer_get_timer_frequency();
		__ASSERT(sleeptimer_freq >= CONFIG_SYS_CLOCK_TICKS_PER_SEC,
			 "SleepTimer Frequency Not suitable for wakeup Timer");

		last_update_lftick = sl_sleeptimer_get_tick_count();
		total_slept_lf_ticks = 0;
		sl_power_manager_sleep();

		if (isContextSwitchRequired == true) {
			isContextSwitchRequired = false;
			/* To invoke the scheduler immediately */
			k_yield();
		}

		/* Remove sleep level limit */
		sl_power_manager_remove_em_requirement(energy_mode);
		break;

	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}

	LOG_DBG("SoC leaving power state %d", state);

	/* Restore RTCC IRQ priority */
	z_arm_irq_priority_set(RTCC_IRQn, (rtcc_prio - 1), 0);
#else

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
		break;
	case PM_STATE_STANDBY:
		EMU_EnterEM3(true);
		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}

	LOG_DBG("SoC leaving power state %d", state);

	/* Clear PRIMASK */
	__enable_irq();

#endif
}

/* Handle SOC specific activity after Low Power Mode Exit */
__weak void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);
}

#ifdef CONFIG_SOC_GECKO_DEV_INIT

/* Function to check any ready threads are scheduled */
bool ConfirmSleepModeStatus(void)
{
	bool ret = true;

	if (!z_is_idle_thread_object(_kernel.ready_q.cache)) {
		/* Issue: sysTick Counter is not incrementing if deep sleep bit is set
		 * WorkAround: Clearing the Deep Sleep Bit , incase not cleared by
		 * sl_power_manager_sleep
		 */
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

		isContextSwitchRequired = true;

		ret = false;
	}
	return ret;
}

/* Function called by power manager to determine if system can go back to sleep
 * after a wakeup
 */
bool sl_power_manager_sleep_on_isr_exit(void)
{
	uint32_t slept_lf_ticks;
	uint32_t slept_os_ticks;

	/* Determine how long we slept. */
	slept_lf_ticks = sl_sleeptimer_get_tick_count() - last_update_lftick;
	slept_os_ticks = (slept_lf_ticks * CONFIG_SYS_CLOCK_TICKS_PER_SEC) / sleeptimer_freq;
	last_update_lftick += slept_lf_ticks;

	/* Notify RTOS of how long we slept. */
	if ((total_slept_lf_ticks + slept_lf_ticks) < expected_sleep_lf_ticks) {
		total_slept_lf_ticks += slept_lf_ticks;
	} else {
		slept_os_ticks = ((expected_sleep_lf_ticks - total_slept_lf_ticks) *
				  CONFIG_SYS_CLOCK_TICKS_PER_SEC) /
				 sleeptimer_freq;
		total_slept_lf_ticks = expected_sleep_lf_ticks;
	}

	if (SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) {
		sys_clock_announce(slept_os_ticks);
	}

	/* Have we slept enough ? */
	if (total_slept_lf_ticks >= expected_sleep_lf_ticks) {
		return false;
	}
	/*  Check if we can sleep again */
	return (ConfirmSleepModeStatus() != false);
}

/* Function called by power manager to determine if system can go to sleep
 * or not
 */
bool sl_power_manager_is_ok_to_sleep(void)
{
	return (ConfirmSleepModeStatus() != false);
}

#endif /* CONFIG_SOC_GECKO_DEV_INIT */
