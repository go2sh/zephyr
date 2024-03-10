/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_tc3xx_stm

#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree/clocks.h>

#define TIMER_BASE_ADDR DT_REG_ADDR_BY_IDX(DT_CHOSEN(infineon_tc3xx_stm), 0)

#define TICKS_PER_SEC   CONFIG_SYS_CLOCK_TICKS_PER_SEC
#define CYCLES_PER_SEC  CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
#define CYCLES_PER_TICK (CYCLES_PER_SEC / TICKS_PER_SEC)

#define STM_TIM0   0x10
#define STM_CMP0   0x30
#define STM_COMCON 0x38
#define STM_ICR    0x3C
#define STM_ISCR   0x40
#define STM_TIM0SV 0x50
#define STM_OCS    0xE8

#define STM_CMCON_MSTART0_OFF 8
#define STM_CMCON_MSIZE0_OFF  0
#define STM_ICR_CMP0EN_OFF    0
#define STM_ICR_CMP0OS_OFF    2
#define STM_ISCR_CMP0IRR_OFF  0

#ifdef CONFIG_TICKLESS_KERNEL
static uint64_t last_count;
#endif

static uint32_t get_time32(void)
{
	return *(volatile uint32_t *)(TIMER_BASE_ADDR + STM_TIM0);
}

static uint64_t get_time64(void)
{
	uint64_t time = *(volatile uint32_t *)(TIMER_BASE_ADDR + STM_TIM0SV);
	time |= ((uint64_t) * (volatile uint32_t *)(TIMER_BASE_ADDR + STM_TIM0SV + 4) << 32);
	return time;
}

static uint32_t get_compare()
{
	return *(volatile uint32_t *)(TIMER_BASE_ADDR + STM_CMP0);
}

static void set_compare(uint32_t cmp)
{
	*(volatile uint32_t *)(TIMER_BASE_ADDR + STM_CMP0) = cmp;
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	if (ticks == K_TICKS_FOREVER) {
		// TODO: Handle correctly
		set_compare(get_time32() - 1);
	}

	set_compare(get_time32() + ((uint32_t)ticks * CYCLES_PER_TICK));
}

uint32_t sys_clock_elapsed(void)
{
#ifdef CONFIG_TICKLESS_KERNEL
	/* Return the number of ticks since last announcement */
	return (get_time64() - last_count) / CYCLES_PER_TICK;
#else
	/* Always return 0 for tickful operation */
	return 0;
#endif
}

uint32_t sys_clock_cycle_get_32(void)
{
	/* Return the current counter value */
	return get_time32();
}

uint64_t sys_clock_cycle_get_64(void)
{
	/* Return the current counter value */
	return get_time64();
}

void sys_clock_disable()
{
	volatile uint32_t *icr = (volatile uint32_t *)(TIMER_BASE_ADDR + STM_ICR);
	*icr &= ~(1 << STM_ICR_CMP0OS_OFF);
	irq_disable(DT_IRQ_BY_IDX(DT_CHOSEN(infineon_tc3xx_stm), 0, irq));
}

static void sys_clock_isr()
{
	uint64_t dticks;
	uint64_t now = get_time64();
	uint32_t compare = get_compare();

	// if (unlikely((compare - now) <= compare)) {
	//	return;
	// }
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		set_compare(get_time32() + CYCLES_PER_TICK);
	}

	dticks = (now - last_count) / CYCLES_PER_TICK;

	last_count += dticks * CYCLES_PER_TICK;
	sys_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? (int32_t)dticks : (dticks > 0));
}

static int sys_clock_driver_init(void)
{
	uint32_t reg_val;

	IRQ_CONNECT(DT_IRQ_BY_IDX(DT_CHOSEN(infineon_tc3xx_stm), 0, irq),
		    DT_IRQ_BY_IDX(DT_CHOSEN(infineon_tc3xx_stm), 0, priority), sys_clock_isr, NULL,
		    0);

	const struct device *dev =
		DEVICE_DT_GET(DT_PARENT(DT_CLOCKS_CTLR(DT_CHOSEN(infineon_tc3xx_stm))));
	clock_control_on(dev, (clock_control_subsys_t)DT_NODE_CHILD_IDX(
				      DT_CLOCKS_CTLR(DT_CHOSEN(infineon_tc3xx_stm))));

#ifdef CONFIG_TICKLESS_KERNEL
	/* Initialise internal states */
	last_count = 0;
#endif
	volatile uint32_t *cmcon = (volatile uint32_t *)(TIMER_BASE_ADDR + STM_COMCON);
	volatile uint32_t *icr = (volatile uint32_t *)(TIMER_BASE_ADDR + STM_ICR);
	volatile uint32_t *iscr = (volatile uint32_t *)(TIMER_BASE_ADDR + STM_ISCR);
	*cmcon = (0 << STM_CMCON_MSTART0_OFF) | (31 << STM_CMCON_MSIZE0_OFF);
	*icr &= ~(1 << STM_ICR_CMP0OS_OFF);

	irq_enable(DT_IRQ_BY_IDX(DT_CHOSEN(infineon_tc3xx_stm), 0, irq));

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		set_compare(get_time32() + CYCLES_PER_TICK);
	}
	*iscr |= (1 << STM_ISCR_CMP0IRR_OFF);
	*icr |= (1 << STM_ICR_CMP0EN_OFF);

#if DT_PROP(DT_CHOSEN(infineon_tc3xx_stm), freeze)
	volatile uint32_t *ocs = (volatile uint32_t *)(TIMER_BASE_ADDR + STM_OCS);
	*ocs = 0x12000000;
#endif

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
