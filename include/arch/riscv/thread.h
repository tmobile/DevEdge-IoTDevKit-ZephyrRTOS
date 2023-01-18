/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Per-arch thread definition
 *
 * This file contains definitions for
 *
 *  struct _thread_arch
 *  struct _callee_saved
 *
 * necessary to instantiate instances of struct k_thread.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RISCV_THREAD_H_
#define ZEPHYR_INCLUDE_ARCH_RISCV_THREAD_H_

#ifndef _ASMLANGUAGE
#include <zephyr/types.h>

/*
 * The following structure defines the list of registers that need to be
 * saved/restored when a context switch occurs.
 */
struct _callee_saved {
	ulong_t sp;	/* Stack pointer, (x2 register) */
	ulong_t ra;	/* return address */

	ulong_t s0;	/* saved register/frame pointer */
	ulong_t s1;	/* saved register */
	ulong_t s2;	/* saved register */
	ulong_t s3;	/* saved register */
	ulong_t s4;	/* saved register */
	ulong_t s5;	/* saved register */
	ulong_t s6;	/* saved register */
	ulong_t s7;	/* saved register */
	ulong_t s8;	/* saved register */
	ulong_t s9;	/* saved register */
	ulong_t s10;	/* saved register */
	ulong_t s11;	/* saved register */
};
typedef struct _callee_saved _callee_saved_t;

#if !defined(RV_FP_TYPE)
#ifdef CONFIG_CPU_HAS_FPU_DOUBLE_PRECISION
#define RV_FP_TYPE uint64_t
#else
#define RV_FP_TYPE uint32_t
#endif
#endif

struct z_riscv_fp_context {
	RV_FP_TYPE fa0, fa1, fa2, fa3, fa4, fa5, fa6, fa7;
	RV_FP_TYPE ft0, ft1, ft2, ft3, ft4, ft5, ft6, ft7, ft8, ft9, ft10, ft11;
	RV_FP_TYPE fs0, fs1, fs2, fs3, fs4, fs5, fs6, fs7, fs8, fs9, fs10, fs11;
	uint32_t fcsr;
};
typedef struct z_riscv_fp_context z_riscv_fp_context_t;

#define PMP_M_MODE_SLOTS 8	/* 8 is plenty enough for m-mode */

struct _thread_arch {
#ifdef CONFIG_FPU_SHARING
	struct z_riscv_fp_context saved_fp_context;
	uint8_t exception_depth;
#endif
#ifdef CONFIG_USERSPACE
	ulong_t priv_stack_start;
	ulong_t u_mode_pmpaddr_regs[CONFIG_PMP_SLOTS];
	ulong_t u_mode_pmpcfg_regs[CONFIG_PMP_SLOTS / sizeof(ulong_t)];
	unsigned int u_mode_pmp_domain_offset;
	unsigned int u_mode_pmp_end_index;
	unsigned int u_mode_pmp_update_nr;
#endif
#ifdef CONFIG_PMP_STACK_GUARD
	unsigned int m_mode_pmp_end_index;
	ulong_t m_mode_pmpaddr_regs[PMP_M_MODE_SLOTS];
	ulong_t m_mode_pmpcfg_regs[PMP_M_MODE_SLOTS / sizeof(ulong_t)];
#endif
};

typedef struct _thread_arch _thread_arch_t;

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_RISCV_THREAD_H_ */
