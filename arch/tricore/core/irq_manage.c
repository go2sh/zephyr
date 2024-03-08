/*
 * Copyright (c) 2024 Infineon Technologies AG
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/interrupt_controller/intc_tc_ir.h>

void arch_irq_enable(unsigned int irq)
{
	intc_tc_ir_irq_enable(irq);
}

void arch_irq_disable(unsigned int irq)
{
	intc_tc_ir_irq_disable(irq);
}

int arch_irq_is_enabled(unsigned int irq)
{
	return intc_tc_ir_irq_is_enabled(irq);
}

void z_tricore_irq_priority_set(unsigned int irq, unsigned int prio)
{
	intc_tc_ir_irq_set_priority(irq, prio);
}

void z_irq_spurious(const void *unused)
{
	ARG_UNUSED(unused);
	z_fatal_error(K_ERR_SPURIOUS_IRQ, NULL);
}
