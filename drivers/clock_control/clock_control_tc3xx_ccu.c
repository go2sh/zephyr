/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_tc3xx_ccu

#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>
#include "clock_control_tc3xx_ccu.h"

BUILD_ASSERT(DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(fsource0)), DT_NODELABEL(fback)) ||
		     DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(fsource0)), DT_NODELABEL(fpll0)),
	     "Invalid fsource0 config");
BUILD_ASSERT(DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(fsource1)), DT_NODELABEL(fback)) ||
		     DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(fsource1)), DT_NODELABEL(fpll1)),
	     "Invalid fsource1 config");
BUILD_ASSERT(DT_PROP(DT_NODELABEL(fsource1), clock_div) == 1 ||
		     (DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(fsource1)), DT_NODELABEL(fpll1)) &&
		      DT_PROP(DT_NODELABEL(fsource1), clock_div) <= 2),
	     "Invalid div for source1");
BUILD_ASSERT(DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(fsource2)), DT_NODELABEL(fback)) ||
		     DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(fsource2)), DT_NODELABEL(fpll2)),
	     "Invalid fsource2 config");
BUILD_ASSERT(!CCU_OSC_IN_USE || DT_NODE_HAS_STATUS(DT_NODELABEL(fosc), okay), "osc must be okay");

enum tc3xx_frequencies {
	DT_FOREACH_CHILD_SEP(DT_N_S_clocks, CCU_FIXED_CLOCK_FREQ, (, )),
	DT_FOREACH_CHILD_SEP(DT_NODELABEL(ccu), DT_FREQ, (, )),
};

static inline int clock_control_tc3xx_ccu_osc_init(mm_reg_t osccon, uint32_t freq)
{

	sys_write32((sys_read32(osccon) & ~SCU_OSCCON_MODE_MSK & ~SCU_OSCCON_OSCVAL_MSK) |
			    (((freq / 1000000U) - 15) << SCU_OSCCON_OSCVAL_POS),
		    osccon);

	if (!WAIT_FOR(
		    (sys_read32(osccon) & (BIT(SCU_OSCCON_PLLLV_POS) | BIT(SCU_OSCCON_PLLHV_POS))),
		    10000, k_busy_wait(1))) {
		return -ETIMEDOUT;
	}

	return 0;
}

static inline void clock_control_tc3xx_ccu_pll_init(mm_reg_t pllcon0, uint8_t p_div, uint8_t n_div,
						    bool divby, uint8_t insel)
{
	sys_write32((divby << SCU_PLLCON0_DIVBY_POS) | (0 << SCU_PLLCON0_MODEN_POS) | (n_div << 9) |
			    BIT(SCU_PLLCON0_PLLPWD_POS) | BIT(SCU_PLLCON0_RESLD_POS) |
			    (p_div << 24) | (insel << 30),
		    pllcon0);
}

static inline int clock_control_tc3xx_ccu_pll_power(mem_addr_t pllcon, mem_addr_t pllstat,
						    bool enabled)
{
	uint32_t old_con = sys_read32(pllcon);
	sys_write32((old_con & ~BIT(SCU_PLLCON0_PLLPWD_POS)) | (enabled << SCU_PLLCON0_PLLPWD_POS),
		    pllcon);

	if (!(old_con & SCU_PLLCON0_PLLPWD_POS)) {
		/* Wait 1ms to stabilize pll after been disabled. */
		k_busy_wait(1000);
	}
	if (!WAIT_FOR((sys_read32(pllstat) & BIT(SCU_PLLSTAT_PWDSTAT_POS)) ==
			      (enabled ? 0 : BIT(SCU_PLLSTAT_PWDSTAT_POS)),
		      1000, k_busy_wait(100))) {
		return -EIO;
	}

	return 0;
}

static inline int clock_control_tc3xx_ccu_pll_devider(mm_reg_t pllcon1, mm_reg_t pllstat,
						      uint8_t k2_div, uint8_t k3_div, bool has_k3)
{
	uint32_t mask = (BIT(SCU_PLLSTAT_K2RDY_POS) | (has_k3 ? BIT(SCU_PLLSTAT_K3RDY_POS) : 0));
	if (!WAIT_FOR((sys_read32(pllstat) & mask) == mask, 10000, k_busy_wait(1))) {
		return -EIO;
	}
	sys_write32((k2_div << 0) | (k3_div << 8), pllcon1);

	return 0;
}

static inline int clock_control_tc3xx_ccu_pll_wait_lock(mm_reg_t sysstat, mm_reg_t peristat)
{
	if (!WAIT_FOR((sys_read32(sysstat) & BIT(SCU_PLLSTAT_LOCK_POS) &&
		       sys_read32(peristat) & BIT(SCU_PLLSTAT_LOCK_POS)),
		      100, k_busy_wait(1))) {
		return -EIO;
	}
	return 0;
}

static inline int clock_control_tc3xx_ccu_init_divider()
{
	CCU_WAIT_FOR_UNLOCK_OR_ERR(0)
	sys_write32((1 << 26) | (1 << 24) | (1 << 20) | (1 << 16) | (1 << 8) |
			    (100000000UL / CCU_FREQ(DT_NODELABEL(fstm)) << 0),
		    CCU_REG_ADDR(0));
	CCU_WAIT_FOR_UNLOCK_OR_ERR(5)
	sys_write32(BIT(SCU_CCUCON_UP_POS), CCU_REG_ADDR(5));
	CCU_WAIT_FOR_UNLOCK_OR_ERR(5)
	CCU_WAIT_FOR_UNLOCK_OR_ERR(1)
	sys_write32(0, CCU_REG_ADDR(1));
	CCU_WAIT_FOR_UNLOCK_OR_ERR(2)
	sys_write32(0, CCU_REG_ADDR(2));

#define CCU_CPU_INIT(x)                                                                            \
	CCU_WAIT_FOR_UNLOCK_OR_ERR(CCU_REG_NR(CCU_SOURCE_CLOCK(x)))                                \
	sys_write32(0, CCU_REG_ADDR(CCU_REG_NR(DT_CLOCKS_CTLR(x))));
	DT_FOREACH_CHILD(DT_N_S_cpus, CCU_CPU_INIT);

	return 0;
}

static inline int clock_control_tc3xx_ccu_select_clock(uint8_t clksel)
{
	CCU_WAIT_FOR_UNLOCK_OR_ERR(0)
	sys_write32((sys_read32(CCU_REG_ADDR(0)) & ~SCU_CCUCON0_CLKSEL_MSK) |
			    BIT(SCU_CCUCON_UP_POS) | (clksel << SCU_CCUCON0_CLKSEL_POS),
		    CCU_REG_ADDR(0));
	/* Wait for new clock config to be active */
	CCU_WAIT_FOR_UNLOCK_OR_ERR(0)
	return 0;
}

static inline int clock_control_tc3xx_ccu_set_divider()
{
	CCU_WAIT_FOR_UNLOCK_OR_ERR(0)
	sys_write32((sys_read32(CCU_REG_ADDR(0)) & SCU_CCUCON0_CLKSEL_MSK) | CCU_REG_VALUE(0) |
			    BIT(SCU_CCUCON_UP_POS),
		    CCU_REG_ADDR(0));

#define CCU_CPU_SET(x)                                                                             \
	CCU_WAIT_FOR_UNLOCK_OR_ERR(CCU_REG_NR(CCU_SOURCE_CLOCK(x)))                                \
	sys_write32(CCU_VALUE(DT_CLOCKS_CTLR(x)), CCU_REG_ADDR(CCU_REG_NR(CCU_SOURCE_CLOCK(x))));
	DT_FOREACH_CHILD_STATUS_OKAY(DT_N_S_cpus, CCU_CPU_SET);

	return 0;
}

static int clock_control_tc3xx_ccu_on(const struct device *dev, clock_control_subsys_t sys)
{
	switch ((uintptr_t)sys) {
#define CCU_CLOCK_ON(n)                                                                            \
	case DT_NODE_CHILD_IDX(n):                                                                 \
		if (!DT_NODE_HAS_STATUS(n, okay)) {                                                \
			return -EINVAL;                                                            \
		}                                                                                  \
		CCU_WAIT_FOR_UNLOCK_OR_ERR(CCU_REG_NR(n))                                          \
		sys_write32((sys_read32(CCU_REG_ADDR(CCU_REG_NR(n))) & ~CCU_MASK(n)) |             \
				    CCU_VALUE(n) | CCU_REG_UP_FLAG(n),                             \
			    CCU_REG_ADDR(CCU_REG_NR(n)));                                          \
		break;
		CCU_CLOCK_FOREACH(CCU_CLOCK_ON)
	default:
		return -EINVAL;
	}

	return 0;
}

static int clock_control_tc3xx_ccu_off(const struct device *dev, clock_control_subsys_t sys)
{
	switch ((uintptr_t)sys) {
#define CCU_CLOCK_OFF_NOTSUP(n)                                                                    \
	case DT_NODE_CHILD_IDX(n):                                                                 \
		return -ENOTSUP;
#define CCU_CLOCK_OFF_OK(n)                                                                        \
	case DT_NODE_CHILD_IDX(n):                                                                 \
		if (!DT_NODE_HAS_STATUS(n, okay)) {                                                \
			return -EINVAL;                                                            \
		}                                                                                  \
		CCU_WAIT_FOR_UNLOCK_OR_ERR(CCU_REG_NR(n))                                          \
		sys_write32((sys_read32(CCU_REG_ADDR(CCU_REG_NR(n))) & ~CCU_MASK(n)) |             \
				    CCU_VALUE(n) | CCU_REG_UP_FLAG(n),                             \
			    CCU_REG_ADDR(CCU_REG_NR(n)));                                          \
		break;
#define CCU_CLOCK_OFF(n)                                                                           \
	COND_CODE_1(DT_NODE_HAS_PROP(n, cpu_field), (CCU_CLOCK_OFF_NOTSUP(n)),                     \
		    (COND_CODE_1(CCU_IS_MANDATORY(n), (CCU_CLOCK_OFF_NOTSUP(n)),                   \
				 (CCU_CLOCK_OFF_OK(n)))))
		CCU_CLOCK_FOREACH(CCU_CLOCK_OFF)
	default:
		return -EINVAL;
	}

	return 0;
}

static enum clock_control_status clock_control_tc3xx_ccu_get_status(const struct device *dev,
								    clock_control_subsys_t sys)
{
	switch ((uintptr_t)sys) {
#define CCU_GET_STATUS_CASE(n)                                                                     \
	case DT_NODE_CHILD_IDX(n):                                                                 \
		if (!DT_NODE_HAS_STATUS(n, okay)) {                                                \
			return CLOCK_CONTROL_STATUS_UNKNOWN;                                       \
		} else if (DT_NODE_HAS_PROP(n, cpu_field) ||                                       \
			   (sys_read32(CCU_REG_ADDR(CCU_REG_NR(n))) & CCU_MASK(n)) != 0) {         \
			return CLOCK_CONTROL_STATUS_ON;                                            \
		} else {                                                                           \
			return CLOCK_CONTROL_STATUS_OFF;                                           \
		}                                                                                  \
		break;

		CCU_CLOCK_FOREACH(CCU_GET_STATUS_CASE)
	default:
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}
}

static int clock_control_tc3xx_ccu_get_rate(const struct device *dev, clock_control_subsys_t sys,
					    uint32_t *rate)
{
	switch ((uintptr_t)sys) {
#define CCU_GET_RATE_CASE(n)                                                                       \
	case DT_NODE_CHILD_IDX(n):                                                                 \
		if (!DT_NODE_HAS_STATUS(n, okay)) {                                                \
			return -EINVAL;                                                            \
		}                                                                                  \
		if (DT_NODE_HAS_PROP(n, cpu_field) ||                                              \
		    (sys_read32(CCU_REG_ADDR(CCU_REG_NR(n))) & CCU_MASK(n)) != 0) {                \
			*rate = CCU_FREQ(n);                                                       \
		} else {                                                                           \
			*rate = 0;                                                                 \
		}                                                                                  \
		break;
		CCU_CLOCK_FOREACH(CCU_GET_RATE_CASE)
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int clock_control_tc3xx_ccu_init(const struct device *dev)
{
	int ret;

	/* Select backup clock und set STM freq to back-up clock */
	clock_control_tc3xx_ccu_init_divider();

	/* Enable osc if requrired */
	if (CCU_OSC_IN_USE) {
		ret = clock_control_tc3xx_ccu_osc_init(
			DT_INST_REG_ADDR(0) + SCU_OSCCON,
			DT_PROP(DT_NODELABEL(fosc), clock_frequency));
		if (ret) {
			return ret;
		}
	}

	if (CCU_USE_PLL) {
		/* Enable pll */
		ret = clock_control_tc3xx_ccu_pll_power(DT_INST_REG_ADDR(0) + SCU_SYSPLLCON0,
							DT_INST_REG_ADDR(0) + SCU_SYSPLLSTAT, true);
		ret |= clock_control_tc3xx_ccu_pll_power(DT_INST_REG_ADDR(0) + SCU_PERPLLCON0,
							 DT_INST_REG_ADDR(0) + SCU_PERPLLSTAT,
							 true);
		if (ret) {
			return ret;
		}

		/* Configure pll parameters*/
		clock_control_tc3xx_ccu_pll_init(
			DT_INST_REG_ADDR(0) + SCU_SYSPLLCON0,
			DT_PROP(DT_NODELABEL(sys_pll), p_div) - 1,
			DT_PROP(DT_NODELABEL(peri_pll), n_div) - 1, 0,
			DT_NODE_CHILD_IDX(DT_CLOCKS_CTLR(DT_NODELABEL(sys_pll))));
		clock_control_tc3xx_ccu_pll_init(DT_INST_REG_ADDR(0) + SCU_PERPLLCON0,
						 DT_PROP(DT_NODELABEL(peri_pll), p_div) - 1,
						 DT_PROP(DT_NODELABEL(peri_pll), n_div) - 1,
						 DT_PROP(DT_NODELABEL(peri_pll), divby), 0);

		/* Set k diver values */
		ret = clock_control_tc3xx_ccu_pll_devider(
			DT_INST_REG_ADDR(0) + SCU_SYSPLLCON1, DT_INST_REG_ADDR(0) + SCU_SYSPLLSTAT,
			DT_PROP(DT_NODELABEL(fpll0), clock_div) - 1 + 3, 0, false);
		ret |= clock_control_tc3xx_ccu_pll_devider(
			DT_INST_REG_ADDR(0) + SCU_PERPLLCON1, DT_INST_REG_ADDR(0) + SCU_PERPLLSTAT,
			DT_PROP(DT_NODELABEL(fpll1), clock_div) - 1,
			DT_PROP(DT_NODELABEL(fpll2), clock_div) /
					COND_CODE_1(DT_PROP(DT_NODELABEL(peri_pll), divby), (2),
						    (16)) -
				1,
			true);
		if (ret) {
			return ret;
		}

		ret = clock_control_tc3xx_ccu_pll_wait_lock(DT_INST_REG_ADDR(0) + SCU_SYSPLLSTAT,
							    DT_INST_REG_ADDR(0) + SCU_PERPLLSTAT);
		if (ret) {
			return ret;
		}
	} else {
		/* Disable pll power */
		ret = clock_control_tc3xx_ccu_pll_power(DT_INST_REG_ADDR(0) + SCU_SYSPLLCON0,
							DT_INST_REG_ADDR(0) + SCU_SYSPLLSTAT,
							false);
		ret |= clock_control_tc3xx_ccu_pll_power(DT_INST_REG_ADDR(0) + SCU_PERPLLCON0,
							 DT_INST_REG_ADDR(0) + SCU_PERPLLSTAT,
							 false);
		if (ret) {
			return ret;
		}
	}

	ret = clock_control_tc3xx_ccu_set_divider();
	if (ret) {
		return -EIO;
	}

	/* Select clock for operation */
	ret = clock_control_tc3xx_ccu_select_clock(
		DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(fsource0)), DT_NODELABEL(fpll0)));
	if (ret) {
		return ret;
	}

	if (DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(fsource0)), DT_NODELABEL(fpll0))) {
		/* Frequency stepping, to avoid load jumps */
		int32_t i;
		for (i = 2; i >= 0; i--) {
			ret = clock_control_tc3xx_ccu_pll_devider(
				DT_INST_REG_ADDR(0) + SCU_SYSPLLCON1,
				DT_INST_REG_ADDR(0) + SCU_SYSPLLSTAT,
				DT_PROP(DT_NODELABEL(fpll0), clock_div) - 1 + i, 0, false);
			if (ret) {
				return ret;
			}
			/* Wait for freq to be settled, scale it due to stm div mis match */
			k_busy_wait(100 / (i + 1));
		}
	}

	return 0;
}

static const struct clock_control_driver_api clock_control_tc3xx_ccu_api = {
	.on = clock_control_tc3xx_ccu_on,
	.off = clock_control_tc3xx_ccu_off,
	.get_rate = clock_control_tc3xx_ccu_get_rate,
	.get_status = clock_control_tc3xx_ccu_get_status,
};

DEVICE_DT_INST_DEFINE(0, &clock_control_tc3xx_ccu_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_tc3xx_ccu_api);
