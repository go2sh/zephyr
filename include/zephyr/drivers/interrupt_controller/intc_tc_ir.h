/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_TC_IR_H_
#define ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_TC_IR_H_

#define IR_BASE	    DT_REG_ADDR_BY_IDX(DT_INST(0, infineon_tc_ir), 0)
#define SRC_BASE	DT_REG_ADDR_BY_IDX(DT_INST(0, infineon_tc_ir), 1)

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>
#include <zephyr/device.h>

void intc_tc_ir_irq_enable(unsigned int irq);

void intc_tc_ir_irq_disable(unsigned int irq);

bool intc_tc_ir_irq_is_enabled(unsigned int irq);

bool intc_tc_ir_irq_is_pending(unsigned int irq);

void intc_tc_ir_irq_clear_pending(unsigned int irq);

void intc_tc_ir_irq_set_priority(
	unsigned int irq, unsigned int prio);

unsigned int intc_tc_ir_get_active(void);

void intc_tc_ir_raise_gpsr(unsigned char group, unsigned char channel);


#endif
#endif