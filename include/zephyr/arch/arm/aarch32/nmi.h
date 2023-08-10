/**
 * @file
 *
 * @brief ARM AArch32 NMI routines
 */

/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_NMI_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_NMI_H_

#ifndef _ASMLANGUAGE
#ifdef CONFIG_RUNTIME_NMI
extern void z_arm_nmi_init(void);
extern void z_arm_nmi_set_handler(void (*pHandler)(void));
#define NMI_INIT() z_arm_nmi_init()
#else
#define NMI_INIT()
#endif
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_NMI_H_ */
