/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <ksched.h>
#include <zephyr/irq.h>

volatile struct {
	arch_cpustart_t fn;
	void *arg;
} riscv_cpu_init[CONFIG_MP_MAX_NUM_CPUS];

volatile uintptr_t riscv_cpu_wake_flag;
volatile void *riscv_cpu_sp;

void arch_start_cpu(int cpu_num, k_thread_stack_t *stack, int sz,
		    arch_cpustart_t fn, void *arg)
{
	riscv_cpu_init[cpu_num].fn = fn;
	riscv_cpu_init[cpu_num].arg = arg;

	riscv_cpu_sp = Z_KERNEL_STACK_BUFFER(stack) + sz;
	riscv_cpu_wake_flag = _kernel.cpus[cpu_num].arch.hartid;

	while (riscv_cpu_wake_flag != 0U) {
		;
	}
}

void z_riscv_secondary_cpu_init(int hartid)
{
	unsigned int i;
	unsigned int cpu_num = 0;

	for (i = 0; i < CONFIG_MP_MAX_NUM_CPUS; i++) {
		if (_kernel.cpus[i].arch.hartid == hartid) {
			cpu_num = i;
		}
	}
	csr_write(mscratch, &_kernel.cpus[cpu_num]);
#ifdef CONFIG_SMP
	_kernel.cpus[cpu_num].arch.online = true;
#endif
#ifdef CONFIG_THREAD_LOCAL_STORAGE
	__asm__("mv tp, %0" : : "r" (z_idle_threads[cpu_num].tls));
#endif
#if defined(CONFIG_RISCV_SOC_INTERRUPT_INIT)
	soc_interrupt_init();
#endif
#ifdef CONFIG_RISCV_PMP
	z_riscv_pmp_init();
#endif
#ifdef CONFIG_SMP
	irq_enable(RISCV_MACHINE_SOFT_IRQ);
#endif
	riscv_cpu_init[cpu_num].fn(riscv_cpu_init[cpu_num].arg);
}

#ifdef CONFIG_SMP
static uint32_t *get_hart_msip(int hart_id)
{
	return (uint32_t *)(unsigned long)(RISCV_MSIP_BASE + (hart_id * 4));
}

void arch_sched_ipi(void)
{
	unsigned int key;
	uint32_t i;
	uint8_t id;

	key = arch_irq_lock();

	id = _current_cpu->id;
	unsigned int num_cpus = arch_num_cpus();

	for (i = 0U; i < num_cpus; i++) {
		if (i != id && _kernel.cpus[i].arch.online) {
			volatile uint32_t *r = get_hart_msip(_kernel.cpus[i].arch.hartid);
			*r = 1U;
		}
	}

	arch_irq_unlock(key);
}

static void sched_ipi_handler(const void *unused)
{
	ARG_UNUSED(unused);

	MSIP(csr_read(mhartid)) = 0;

	atomic_val_t pending_ipi = atomic_clear(&cpu_pending_ipi[_current_cpu->id]);

	if (pending_ipi & IPI_SCHED) {
		z_sched_ipi();
	}
#ifdef CONFIG_FPU_SHARING
	if (pending_ipi & IPI_FPU_FLUSH) {
		/* disable IRQs */
		csr_clear(mstatus, MSTATUS_IEN);
		/* perform the flush */
		z_riscv_flush_local_fpu();
		/*
		 * No need to re-enable IRQs here as long as
		 * this remains the last case.
		 */
	}
#endif
}

static int riscv_smp_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	IRQ_CONNECT(RISCV_MACHINE_SOFT_IRQ, 0, sched_ipi_handler, NULL, 0);
	irq_enable(RISCV_MACHINE_SOFT_IRQ);

	return 0;
}

SYS_INIT(riscv_smp_init, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif /* CONFIG_SMP */
