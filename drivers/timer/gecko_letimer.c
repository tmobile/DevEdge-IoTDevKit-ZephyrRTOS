/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_letimer.h"
#include <zephyr/sys/printk.h>

#define DT_DRV_COMPAT silabs_gecko_letimer

#define LFXO_LFRCO_FREQ 32768U

#define PRESCALER DT_PROP(DT_DRV_INST(0), prescaler)

#define PRESCALED_FREQ (LFXO_LFRCO_FREQ / PRESCALER)


#define CYC_PER_TICK ((uint32_t)((uint64_t)(PRESCALED_FREQ) \
	  / (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC))

BUILD_ASSERT(CONFIG_SYS_CLOCK_TICKS_PER_SEC >= 0,
	"CONFIG_SYS_CLOCK_TICKS_PER_SEC must be a positive value");
BUILD_ASSERT(CONFIG_SYS_CLOCK_TICKS_PER_SEC <= PRESCALED_FREQ,
	"CONFIG_SYS_CLOCK_TICKS_PER_SEC must be lower than oscillator frequency");
/* For now; TODO: handle non-power-of-two TICKS_PER_SEC values gracefully */
BUILD_ASSERT(!((CONFIG_SYS_CLOCK_TICKS_PER_SEC - 1) & CONFIG_SYS_CLOCK_TICKS_PER_SEC),
	"CONFIG_SYS_CLOCK_TICKS_PER_SEC must be a power of two with Gecko LETIMER");

#define TICKLESS (IS_ENABLED(CONFIG_TICKLESS_KERNEL))

#define REP_MODE (IS_ENABLED(CONFIG_GECKO_LETIMER_USE_REP))

#ifdef _SILICON_LABS_32B_SERIES_2
#define COUNTER_BIT_WIDTH 24U
#else
#define COUNTER_BIT_WIDTH 16U
#endif

#define COUNTER_SPAN BIT(COUNTER_BIT_WIDTH)
#define COUNTER_MAX (COUNTER_SPAN - 1U)
#define COUNTER_HALF_SPAN (COUNTER_SPAN / 2U)
#define MAX_TICKS ((COUNTER_MAX / CYC_PER_TICK) - 1)
#define MAX_CYCLES (MAX_TICKS * CYC_PER_TICK)

/*
 * This local variable holds the amount of elapsed cycles
 * that have been announced to the kernel.
 */
static uint32_t announced_cycles;

/*
 * This local variable holds the total amount of elapsed cycles.
 */
static uint32_t cycle_count;

/*
 * This local variable holds the tick count at last interrupt
 */
static volatile uint32_t last_count;

/*
 * This local variable holds the repeat count at last interrupt
 */
static volatile uint32_t last_rep;

static void gecko_letimer_isr(void *arg)
{
	uint32_t dticks;

#ifdef CONFIG_TICKLESS_KERNEL
	/* Add the last set count to the total */
	cycle_count += last_count;
	/* Default last count to the initial timer value */
	last_count = LETIMER_TopGet(LETIMER0) + 1;

#if REP_MODE
	if (LETIMER_IntGet(LETIMER0) & LETIMER_IF_REP0) {
		LETIMER_RepeatSet(LETIMER0, 0, 1);
		LETIMER_RepeatSet(LETIMER0, 1, 0);
		LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
	}
#endif
	/* Calculated number of ticks elapsed/tick delta */
	dticks = (cycle_count - announced_cycles) / CYC_PER_TICK;
	/* Update count of announced cycles */
	announced_cycles += dticks * CYC_PER_TICK;

	/* Clear interrupts */
	LETIMER_IntClear(LETIMER0, LETIMER_IntGet(LETIMER0));

	/* Announce elapsed ticks */
	sys_clock_announce(dticks);
#else
	uint32_t elapsed_cycles;

	elapsed_cycles = LETIMER_TopGet(LETIMER0) + 1;
	cycle_count += elapsed_cycles;
	dticks = (cycle_count - announced_cycles) / CYC_PER_TICK;
	announced_cycles += dticks * CYC_PER_TICK;

	LETIMER_IntClear(LETIMER0, LETIMER_IntGet(LETIMER0));

	sys_clock_announce(dticks);
#endif
}


void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	int32_t delay;

	if (!TICKLESS) {
		return;
	}

	if (idle && ticks == K_TICKS_FOREVER) {
		/* Disable the timer */
		LETIMER_Enable(LETIMER0, false);
		return;
	}

	/* Translate K_TICKS_FOREVER to MAX_TICKS */
	ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : ticks;

#if !REP_MODE
	/* Bound ticks between 1 and MAX_TICKS */
	ticks = CLAMP(ticks, 1, (int32_t) MAX_TICKS);
	/* Convert ticks to cycles */
	delay = ticks * CYC_PER_TICK;
#else
	ticks = CLAMP(ticks, 1, (int32_t) (MAX_TICKS * 2 * UINT8_MAX + 1));
	if (ticks > MAX_TICKS) {
		int reps;
		uint8_t rep0, rep1;

		reps = ticks / MAX_TICKS;
		rep0 = CLAMP(reps - 1, 1, UINT8_MAX);
		rep1 = CLAMP(reps - 256, 0, UINT8_MAX);
		LETIMER_RepeatSet(LETIMER0, 0, rep0);
		LETIMER_RepeatSet(LETIMER0, 1, rep1);
		LETIMER_IntDisable(LETIMER0, LETIMER_IF_UF);
		delay = LETIMER_TopGet(LETIMER0) * (rep0 + rep1);
		last_rep = rep0 + rep1;
		LETIMER0->CTRL =
			(LETIMER0->CTRL & ~_LETIMER_CTRL_REPMODE_MASK) | letimerRepeatBuffered;
	} else {
		LETIMER0->CTRL = (LETIMER0->CTRL & ~_LETIMER_CTRL_REPMODE_MASK);
		LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
		delay = ticks * CYC_PER_TICK;
	}
#endif

#if !REP_MODE
	/* Set timer to calculated number of cycles */
	LETIMER_CounterSet(LETIMER0, delay - 1);
	/* Update the last_count value with amound of cycles set */
	last_count = CLAMP(delay, 1, COUNTER_SPAN);
#else
	LETIMER_CounterSet(LETIMER0, delay <= COUNTER_MAX ? delay - 1 : LETIMER_TopGet(LETIMER0));
	last_count = delay;
#endif

	if (!(LETIMER0->STATUS & LETIMER_STATUS_RUNNING)) {
		LETIMER_Enable(LETIMER0, true);
	}

}

uint32_t sys_clock_elapsed(void)
{
	if (!TICKLESS) {
		return 0;
	}

	if (last_count <= COUNTER_MAX) {
		return ((last_count - LETIMER_CounterGet(LETIMER0)))  / CYC_PER_TICK;
	}

	int curr_rep = LETIMER_RepeatGet(LETIMER0, 0) + LETIMER_RepeatGet(LETIMER0, 1);
	int el_rep = last_rep - curr_rep;

	return LETIMER_TopGet(LETIMER0) * (el_rep + 1) - LETIMER_CounterGet(LETIMER0);
}

uint32_t sys_clock_cycle_get_32(void)
{
	return sys_clock_elapsed() * CYC_PER_TICK + announced_cycles;
}

void sys_clock_idle_exit(void)
{
	LETIMER_Enable(LETIMER0, true);
}

void sys_clock_disable(void)
{
	LETIMER_Enable(LETIMER0, false);
}

static int sys_clock_driver_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	LETIMER_Init_TypeDef letimer_init = LETIMER_INIT_DEFAULT;

	letimer_init.enable = false;

	LETIMER_IntDisable(LETIMER0, 0xFFFFFFFF);
	LETIMER_IntClear(LETIMER0, 0xFFFFFFFF);

	CMU_ClockEnable(cmuClock_HFLE, true);

#ifdef CONFIG_GECKO_LETIMER_USE_LFXO
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
#else
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
#endif /* CONFIG_GECKO_LETIMER_USE_LFXO */

	CMU_ClockEnable(cmuClock_LETIMER0, true);

	CMU_ClockPrescSet(cmuClock_LETIMER0, PRESCALER);

#if REP_MODE
	LETIMER_RepeatSet(LETIMER0, 0, 1);
#endif

	LETIMER_Init(LETIMER0, &letimer_init);

	if (!TICKLESS) {
		LETIMER_TopSet(LETIMER0, CYC_PER_TICK - 1);
		LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
	} else {
		LETIMER_TopSet(LETIMER0, ((1 << COUNTER_BIT_WIDTH) - 1));
#if !REP_MODE
		LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
#else
		LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF | LETIMER_IF_REP0);
#endif
	}

	NVIC_ClearPendingIRQ(LETIMER0_IRQn);

	IRQ_CONNECT(LETIMER0_IRQn, DT_IRQ(DT_DRV_INST(0), priority),
		    gecko_letimer_isr, 0, 0);

	irq_enable(LETIMER0_IRQn);

	LETIMER_Enable(LETIMER0, true);

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
