/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RISCV_ARCH_INLINES_H_
#define ZEPHYR_INCLUDE_ARCH_RISCV_ARCH_INLINES_H_

#ifndef _ASMLANGUAGE

#include <kernel_structs.h>
#include "csr.h"

static ALWAYS_INLINE uint32_t arch_proc_id(void)
{
	return csr_read(mhartid);
}

static ALWAYS_INLINE _cpu_t *arch_curr_cpu(void)
{
#if defined(CONFIG_SMP) || defined(CONFIG_USERSPACE)
	return (_cpu_t *)csr_read(mscratch);
#else
	return &_kernel.cpus[0];
#endif
}

#endif /* !_ASMLANGUAGE */
#endif /* ZEPHYR_INCLUDE_ARCH_RISCV_ARCH_INLINES_H_ */
