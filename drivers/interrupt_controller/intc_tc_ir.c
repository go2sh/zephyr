/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/interrupt_controller/intc_tc_ir.h>
#include "intc_tc_ir_priv.h"

#define DT_DRV_COMPAT infineon_tc_ir

void intc_tc_ir_irq_enable(unsigned int irq)
{
	volatile SRCR *src = GET_SRC(irq);
	uint32_t coreId = arch_proc_id();
	SRCR reg = *src;

	reg.B.TOS = GET_TOS(coreId);
	reg.B.SRE = 1;
	*src = reg;
	
}

void intc_tc_ir_irq_disable(unsigned int irq)
{
	volatile SRCR *src = GET_SRC(irq);

	src->B.SRE = 0;
}

bool intc_tc_ir_irq_is_enabled(unsigned int irq)
{
	volatile SRCR *src = GET_SRC(irq);

	return (src->B.SRE==1);
}

bool intc_tc_ir_irq_is_pending(unsigned int irq)
{
	volatile SRCR *src = GET_SRC(irq);

	return (src->B.SRR==1);
}

void intc_tc_ir_irq_clear_pending(unsigned int irq)
{
	volatile SRCR *src = GET_SRC(irq);

	src->B.CLRR = 1;
}

void intc_tc_ir_irq_set_priority(
	unsigned int irq, unsigned int prio)
{
	volatile SRCR *src = GET_SRC(irq);

	src->B.SRPN = prio;
}

unsigned int intc_tc_ir_get_active(void)
{
	uint32_t coreId = arch_proc_id();
	coreId = coreId == 0 ? coreId : coreId + 1;
	uint32_t *lasr = (uint32_t *)(IR_BASE+0x204+coreId*0x10);

	return ((*lasr >> 16) & 0x3FF);
}

void intc_tc_ir_raise_gpsr(unsigned char group, unsigned char channel) {
	volatile SRCR *src = GET_SRC(612 + 8*group + channel);
	src->B.SETR = 1;
}

/**
 * @brief Initialize the tx3xx ir device driver
 */
int intc_tc_ir_init(const struct device *dev)
{
	return 0;
}

DEVICE_DT_INST_DEFINE(0, intc_tc_ir_init, NULL, NULL, NULL,
		      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY, NULL);
