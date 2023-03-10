/*
 *
 * Copyright (c) 2017 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_utils.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include "clock_stm32_ll_common.h"

#if defined(STM32_PLL_ENABLED)

/**
 * @brief Return PLL source
 */
__unused
static uint32_t get_pll_source(void)
{
	if (IS_ENABLED(STM32_PLL_SRC_HSI)) {
		return LL_RCC_PLLSOURCE_HSI;
	} else if (IS_ENABLED(STM32_PLL_SRC_HSE)) {
		return LL_RCC_PLLSOURCE_HSE;
	}

	__ASSERT(0, "Invalid source");
	return 0;
}

/**
 * @brief get the pll source frequency
 */
__unused
uint32_t get_pllsrc_frequency(void)
{
	if (IS_ENABLED(STM32_PLL_SRC_HSI)) {
		return STM32_HSI_FREQ;
	} else if (IS_ENABLED(STM32_PLL_SRC_HSE)) {
		return STM32_HSE_FREQ;
	}

	__ASSERT(0, "Invalid source");
	return 0;
}

/**
 * @brief Set up pll configuration
 */
__unused
void config_pll_sysclock(void)
{
	LL_RCC_PLL_ConfigDomain_SYS(get_pll_source(),
				    pllm(STM32_PLL_M_DIVISOR),
				    STM32_PLL_N_MULTIPLIER,
				    pllp(STM32_PLL_P_DIVISOR));
}

#endif /* defined(STM32_PLL_ENABLED) */

#ifdef STM32_PLLI2S_ENABLED

/**
 * @brief Set up PLL I2S configuration
 */
__unused
void config_plli2s(void)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32f4_plli2s_clock)
	LL_RCC_PLLI2S_ConfigDomain_I2S(get_pll_source(),
				       pllm(STM32_PLLI2S_M_DIVISOR),
				       STM32_PLLI2S_N_MULTIPLIER,
				       plli2sr(STM32_PLLI2S_R_DIVISOR));
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32f412_plli2s_clock)
	LL_RCC_PLL_ConfigDomain_I2S(get_pll_source(),
				       plli2sm(STM32_PLLI2S_M_DIVISOR),
				       STM32_PLLI2S_N_MULTIPLIER,
				       plli2sr(STM32_PLLI2S_R_DIVISOR));
#endif
}

#endif /* STM32_PLLI2S_ENABLED */

/**
 * @brief Activate default clocks
 */
void config_enable_default_clocks(void)
{
	/* Power Interface clock enabled by default */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
}
