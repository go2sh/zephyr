/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_tc3xx_stm

#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree/clocks.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/util.h>

#define TIMER_BASE_ADDR DT_REG_ADDR_BY_IDX(DT_CHOSEN(infineon_system_timer), 0)

#define TICKS_PER_SEC   CONFIG_SYS_CLOCK_TICKS_PER_SEC
#define CYCLES_PER_SEC  CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
#define CYCLES_PER_TICK (CYCLES_PER_SEC / TICKS_PER_SEC)
typedef uint32_t cycle_diff_t;
BUILD_ASSERT(CYCLES_PER_TICK < 4294967295, "ticks per cycle must fit in 32bit");

#define STM_TIM0   0x10
#define STM_CMP0   0x30
#define STM_COMCON 0x38
#define STM_ICR    0x3C
#define STM_ISCR   0x40
#define STM_TIM0SV 0x50
#define STM_OCS    0xE8

#define STM_CMCON_MSTART0 GENMASK(12, 8)
#define STM_CMCON_MSIZE0  GENMASK(4, 0)
#define STM_ICR_CMP0EN    BIT(0)
#define STM_ICR_CMP0IR    BIT(1)
#define STM_ICR_CMP0OS    BIT(2)
#define STM_ISCR_CMP0IRR  BIT(0)

static struct k_spinlock lock;
static uint64_t last_count;
static uint64_t last_ticks;
static uint32_t last_elapsed;

static uint32_t get_time32(void)
{
	return sys_read32(TIMER_BASE_ADDR + STM_TIM0);
}

static uint64_t get_time64(void)
{
	uint64_t time = sys_read32(TIMER_BASE_ADDR + STM_TIM0SV);
	time |= ((uint64_t)sys_read32(TIMER_BASE_ADDR + STM_TIM0SV + 4) << 32);
	return time;
}

static uint32_t get_compare()
{
	return sys_read32(TIMER_BASE_ADDR + STM_CMP0);
}

static void set_compare(uint32_t cmp)
{
	sys_write32(cmp, TIMER_BASE_ADDR + STM_CMP0);
}

static void set_compare_irq(bool enabled)
{
	sys_write32(enabled ? STM_ICR_CMP0EN : 0, TIMER_BASE_ADDR + STM_ICR);
}

static void sys_clock_isr()
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	sys_write32(STM_ISCR_CMP0IRR, TIMER_BASE_ADDR + STM_ISCR);

	uint64_t now = get_time64();
	uint64_t dcycles = now - last_count;
	uint32_t dticks = (cycle_diff_t)dcycles / CYCLES_PER_TICK;

	last_count += (cycle_diff_t)dticks * CYCLES_PER_TICK;
	last_ticks += dticks;
	last_elapsed = 0;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		uint64_t next = last_count + CYCLES_PER_TICK;

		/* Even though we use only the lower 32bits for compare an overflow is
		 * not possible as we make sure that the CYCLES_PER_TICK is smaller than 32 bits*/
		set_compare((uint32_t)next);
	}

	k_spin_unlock(&lock, key);
	sys_clock_announce(dticks);
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	if (ticks == K_TICKS_FOREVER) {
		if (!idle) {
			k_spinlock_key_t key = k_spin_lock(&lock);
			set_compare(get_compare() - 1);
			k_spin_unlock(&lock, key);
		} else {
			set_compare_irq(false);
		}
		return;
	}

	/* Limit ticks to facilitate 32bit devisions and to avoid int32 overflow
	 * with sys_tick_announce call. */
	ticks = CLAMP(ticks, 0, (cycle_diff_t)-1 / 2 / CYCLES_PER_TICK);
	ticks = CLAMP(ticks, 0, INT32_MAX / 2);
	/* Additionally clamp ticks plus last_elapsed to the maximum number of
	 * ticks for a single compare operation. */
	ticks = CLAMP(ticks + last_elapsed, 0, (cycle_diff_t)-1 / CYCLES_PER_TICK);

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t cyc = (last_ticks + ticks) * CYCLES_PER_TICK;

	set_compare_irq(true);
	set_compare((uint32_t)cyc);
	k_spin_unlock(&lock, key);
}

uint32_t sys_clock_elapsed(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t now = get_time64();
	uint64_t dcycles = now - last_count;
	uint32_t dticks = (cycle_diff_t)dcycles / CYCLES_PER_TICK;

	last_elapsed = dticks;
	k_spin_unlock(&lock, key);
	return dticks;
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
	set_compare_irq(false);
}

static int sys_clock_driver_init(void)
{
	uint32_t reg_val;
	const struct device *clkctrl =
		DEVICE_DT_GET(DT_PARENT(DT_CLOCKS_CTLR(DT_CHOSEN(infineon_system_timer))));
	clock_control_subsys_t clk =
		(void *)DT_NODE_CHILD_IDX(DT_CLOCKS_CTLR(DT_CHOSEN(infineon_system_timer)));
	int ret;

	if (!device_is_ready(clkctrl)) {
		return -EIO;
	}

	ret = clock_control_on(clkctrl, clk);
	if (ret) {
		return ret;
	}

	IRQ_CONNECT(DT_IRQ_BY_IDX(DT_CHOSEN(infineon_system_timer), 0, irq),
		    DT_IRQ_BY_IDX(DT_CHOSEN(infineon_system_timer), 0, priority), sys_clock_isr,
		    NULL, 0);

	/* Initialise internal states */
	last_ticks = get_time64() / CYCLES_PER_TICK;
	last_count = last_ticks * CYCLES_PER_TICK;

	/* Set debug freeze if selected */
#if DT_PROP(DT_CHOSEN(infineon_system_timer), freeze)
	sys_write32(0x12000000, TIMER_BASE_ADDR + STM_OCS);
#endif
	/* Set compare window */
	sys_write32(FIELD_PREP(STM_CMCON_MSTART0, 0) | FIELD_PREP(STM_CMCON_MSIZE0, 31),
		    TIMER_BASE_ADDR + STM_COMCON);
	/* Set irq line */
	sys_write32(sys_read32(TIMER_BASE_ADDR + STM_ICR) & ~STM_ICR_CMP0OS,
		    TIMER_BASE_ADDR + STM_ICR);
	/* Clear irq */
	sys_write32(STM_ISCR_CMP0IRR, TIMER_BASE_ADDR + STM_ISCR);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		set_compare((uint32_t)(last_count + CYCLES_PER_TICK));
		set_compare_irq(true);
	}
	irq_enable(DT_IRQ_BY_IDX(DT_CHOSEN(infineon_system_timer), 0, irq));

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
