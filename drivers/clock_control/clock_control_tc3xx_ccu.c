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

#define DT_FREQ(a)      DT_CAT(a, _freq)
#define DT_FREQ_NODE(a) DT_FREQ(DT_NODELABEL(a))
#define DT_ID(a)        DT_DEP_ORD(DT_NODELABEL(a))
#define GET_FIXED_FACTOR(a)                                                                        \
	a##_freq = DT_FREQ(DT_CLOCKS_CTLR(a)) /                                                    \
		   COND_CODE_1(DT_NODE_HAS_PROP(a, clock_div), (DT_PROP(a, clock_div)), (1)) *     \
		   COND_CODE_1(DT_NODE_HAS_PROP(a, clock_mult), (DT_PROP(a, clock_mult)), (1))
#define GET_PLL_FACTOR(a)                                                                          \
	a##_freq = DT_FREQ(DT_CLOCKS_CTLR(a)) * DT_PROP(a, n_div) / DT_PROP(a, p_div)
#define DT_GET_FACTOR(a)                                                                           \
	COND_CODE_1(DT_NODE_HAS_COMPAT(a, infineon_tc3xx_pll_clock), (GET_PLL_FACTOR(a)),          \
		    (GET_FIXED_FACTOR(a)))
#define GET_ENUM_FIXED(a) a##_freq = DT_PROP(a, clock_frequency)

enum tc3xx_frequencies {
	DT_FOREACH_CHILD_SEP(DT_N_S_clocks, GET_ENUM_FIXED, (, )),
	DT_FOREACH_CHILD_SEP(DT_NODELABEL(ccu), DT_GET_FACTOR, (, )),
};

// BUILD_ASSERT(DT_FREQ(DT_NODELABEL(fosc))  >= 20000000, "Freq out of range");
// BUILD_ASSERT(DT_FREQ(DT_NODELABEL(sys_pll))  < 600000000, "Freq out of range");

struct clock_control_tc3xx_ccu_config {
	void *base;
	bool use_osc;
	bool peri_divby;
	uint8_t insel;
	uint8_t sys_p_div;
	uint8_t sys_n_div;
	uint8_t sys_k2_div;
	uint8_t peri_p_div;
	uint8_t peri_n_div;
	uint8_t peri_k2_div;
	uint8_t peri_k3_div;
	uint8_t clksel;
};

static int clock_control_tc3xx_ccu_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_tc3xx_ccu_config *cfg = dev->config;
	volatile uint32_t *ccucon;
	uint32_t clock_id = (uint32_t)sys;
	uint32_t div;

	switch (clock_id) {
	case DT_NODE_CHILD_IDX(DT_NODELABEL(fstm)):
		ccucon = cfg->base + SCU_CCUCON0;
		div = DT_PROP(DT_NODELABEL(fstm), clock_div);
		if (div > 15 || div == 7 || div == 9 || div == 11 || div == 13 || div == 14) {
			return -EINVAL;
		}
		while (*ccucon & BIT(31))
			;
		*ccucon = (*ccucon & ~(BIT_MASK(4) << 0)) | div;
		*ccucon |= BIT(31);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int clock_control_tc3xx_ccu_off(const struct device *dev, clock_control_subsys_t sys)
{

	return 0;
}

static enum clock_control_status clock_control_tc3xx_ccu_get_status(const struct device *dev,
								    clock_control_subsys_t sys)
{

	return CLOCK_CONTROL_STATUS_OFF;
}

static int clock_control_tc3xx_ccu_get_rate(const struct device *dev, clock_control_subsys_t sys,
					    uint32_t *rate)
{
	const struct clock_control_tc3xx_ccu_config *cfg = dev->config;
	uint32_t clock_id = (uint32_t)sys;
	uint32_t div;

	switch (clock_id) {
	case DT_ID(fstm):
		if (1) {
			return DT_FREQ_NODE(fstm);
		}
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline int clock_control_tc3xx_ccu_pll_power(volatile uint32_t *con, volatile uint32_t *stat,
						    bool enabled)
{
	uint32_t old_con = *con;
	*con = (old_con & ~BIT(SCU_PLLCON0_PLLPWD_POS)) | (enabled << SCU_PLLCON0_PLLPWD_POS);

	if (!(old_con & SCU_PLLCON0_PLLPWD_POS)) {
		/* Wait 1ms to stabilize pll after been disabled. */
		k_busy_wait(1000);
	}
	WAIT_FOR(enabled ? !(*stat & BIT(SCU_PLLSTAT_PWDSTAT_POS))
			 : (*stat & BIT(SCU_PLLSTAT_PWDSTAT_POS)),
		 1000, k_busy_wait(100));

	if (enabled ? (*stat & BIT(SCU_PLLSTAT_PWDSTAT_POS))
		    : !(*stat & BIT(SCU_PLLSTAT_PWDSTAT_POS))) {
		return -ETIMEDOUT;
	}

	return 0;
}

static inline int clock_control_tc3xx_ccu_osc_init(volatile uint32_t *osccon, uint32_t freq)
{

	*osccon = (*osccon & ~SCU_OSCCON_MODE_MSK & ~SCU_OSCCON_OSCVAL_MSK) |
		  (((freq / 1000000U) - 15) << SCU_OSCCON_OSCVAL_POS);

	WAIT_FOR((*osccon & (BIT(SCU_OSCCON_PLLLV_POS) | BIT(SCU_OSCCON_PLLHV_POS))), 10000,
		 k_busy_wait(1));
	if (!(*osccon & (BIT(SCU_OSCCON_PLLLV_POS) | BIT(SCU_OSCCON_PLLHV_POS)))) {
		return -ETIMEDOUT;
	}

	return 0;
}

static inline void clock_control_tc3xx_ccu_pll_init(volatile uint32_t *pllcon0, uint8_t p_div,
						    uint8_t n_div, bool divby, uint8_t insel)
{
	*pllcon0 = (divby << SCU_PLLCON0_DIVBY_POS) | (0 << SCU_PLLCON0_MODEN_POS) | (n_div << 9) |
		   BIT(SCU_PLLCON0_PLLPWD_POS) | BIT(SCU_PLLCON0_RESLD_POS) | (p_div << 24) |
		   (insel << 30);
}

static inline int clock_control_tc3xx_ccu_pll_devider(volatile uint32_t *pllcon1,
						      volatile uint32_t *pllstat, uint8_t k2_div,
						      uint8_t k3_div, bool has_k3)
{
	uint32_t mask = (BIT(SCU_PLLSTAT_K2RDY_POS) | (has_k3 ? BIT(SCU_PLLSTAT_K3RDY_POS) : 0));
	WAIT_FOR((*pllstat & mask) == mask, 10000, k_busy_wait(1));
	if ((*pllstat & mask) != mask) {
		return -ETIMEDOUT;
	}
	*pllcon1 = (k2_div << 0) | (k3_div << 8);
}

#define __DT_DIV(a)      DT_PROP(a, clock_div)
#define DT_DIV(a)        __DT_DIV(DT_NODELABEL(a))
#define __DT_DIV_OKAY(a) COND_CODE_1(DT_NODE_HAS_STATUS(a, okay), (__DT_DIV(a)), (0))
#define DT_DIV_OKAY(a)   __DT_DIV_OKAY(DT_NODELABEL(a))
#define __DT_CLOCL_SEL(a)                                                                          \
	COND_CODE_1(                                                                               \
		DT_NODE_HAS_STATUS(a, okay),                                                       \
		(COND_CODE_0(IS_EQ(DT_DEP_ORD(a), DT_DEP_ORD(DT_CLOCKS_CTLR(DT_NODELABEL(fosc)))), \
			     (1), (2))),                                                           \
		(0))
#define DT_CLOCK_SEL(a) __DT_CLOCL_SEL(DT_NODELABEL(a))

static inline int clock_control_tc3xx_ccu_set_divider(void *base)
{
	volatile uint32_t *ccucon;
	ccucon = base + SCU_CCUCON0;
	WAIT_FOR((*ccucon & BIT(SCU_CCUCON_LCK_POS)) == 0, 100, k_busy_wait(1));
	*ccucon = (DT_DIV(fstm) << 0) | (DT_DIV_OKAY(fgtm) << 4) | (DT_DIV(fsri) << 8) | 0 |
		  (DT_DIV(fspb) << 16) | (DT_DIV_OKAY(fbbb) << 20) | (DT_DIV(ffsi) << 24) |
		  (DT_DIV(ffsi2) << 26) | 0;
	ccucon = base + SCU_CCUCON5;
	*ccucon = (DT_DIV_OKAY(fgeth) << 0) | (DT_DIV_OKAY(fmcanh) << 4) |
		  (DT_DIV_OKAY(fadas) << 8) | BIT(SCU_CCUCON_UP_POS);

	ccucon = base + SCU_CCUCON1;
	WAIT_FOR((*ccucon & BIT(SCU_CCUCON_LCK_POS)) == 0, 100, k_busy_wait(1));
	*ccucon = 0;
	WAIT_FOR((*ccucon & BIT(SCU_CCUCON_LCK_POS)) == 0, 100, k_busy_wait(1));
	*ccucon = (DT_DIV_OKAY(fmcan) << 0) | (DT_CLOCK_SEL(fmcan) << 4) |
		  (DT_DIV_OKAY(fi2c) << 8) | (DT_DIV_OKAY(fmsc) << 16) |
		  (DT_CLOCK_SEL(fmsc) << 20) | (DT_DIV_OKAY(fqspi) << 24) |
		  ((DT_DIV(fsource1) == 2 &&
		    DT_SAME_NODE(DT_NODELABEL(fpll1), DT_CLOCKS_CTLR(DT_NODELABEL(fsource1))))
		   << 7);

	ccucon = base + SCU_CCUCON2;
	WAIT_FOR((*ccucon & BIT(SCU_CCUCON_LCK_POS)) == 0, 100, k_busy_wait(1));
	*ccucon = 0;
	WAIT_FOR((*ccucon & BIT(SCU_CCUCON_LCK_POS)) == 0, 100, k_busy_wait(1));
	*ccucon = (DT_DIV_OKAY(fasclinf) << 0) | (DT_DIV_OKAY(fasclins) << 8) |
		  (DT_CLOCK_SEL(fasclins) << 12); /* TODO: Powersave Eray*/

#define dt_div(x) DT_PROP(x, clock_div) - 1
#define func(x)                                                                                    \
	ccucon = (base + SCU_CCUCON6 + 4 * DT_NODE_CHILD_IDX(x));                                  \
	*ccucon = dt_div(DT_CLOCKS_CTLR(x));
	DT_FOREACH_CHILD_STATUS_OKAY(DT_N_S_cpus, func);
}

static inline int clock_control_tc3xx_ccu_pll_wait_lock(volatile uint32_t *sysstat,
							volatile uint32_t *peristat)
{
	WAIT_FOR((*sysstat & BIT(SCU_PLLSTAT_LOCK_POS) && *peristat & BIT(SCU_PLLSTAT_LOCK_POS)),
		 100, k_busy_wait(1));
	if (!(*sysstat & BIT(SCU_PLLSTAT_LOCK_POS) && *peristat & BIT(SCU_PLLSTAT_LOCK_POS))) {
		return -ETIMEDOUT;
	}
	return 0;
}

static int clock_control_tc3xx_ccu_init(const struct device *dev)
{
	const struct clock_control_tc3xx_ccu_config *cfg = dev->config;
	volatile uint32_t *ccucon;
	uint32_t ccucon0;
	int ret;

	/* Select backup clock und set STM freq to back-up clock */
	ccucon = cfg->base + SCU_CCUCON0;
	WAIT_FOR((*ccucon & BIT(SCU_CCUCON_LCK_POS)) == 0, 100, k_busy_wait(1));
	if (*ccucon & BIT(SCU_CCUCON_LCK_POS)) {
		return -ETIMEDOUT;
	}
	*ccucon = (*ccucon & ~SCU_CCUCON0_CLKSEL_MSK & ~GENMASK(3, 0)) | BIT(SCU_CCUCON_UP_POS) |
		  (100000000UL / DT_FREQ(DT_NODELABEL(fstm)) << 0);
	/* Wait until change is executed */
	WAIT_FOR((*ccucon & BIT(SCU_CCUCON_LCK_POS)) == 0, 100, k_busy_wait(1));
	if (*ccucon & BIT(SCU_CCUCON_LCK_POS)) {
		return -ETIMEDOUT;
	}

	/* Enable osc if requrired */
	if (cfg->use_osc) {
		ret = clock_control_tc3xx_ccu_osc_init(
			cfg->base + SCU_OSCCON, DT_PROP(DT_NODELABEL(fosc), clock_frequency));
		if (ret) {
			return ret;
		}
	}

	if (cfg->clksel) {
		/* Enable pll */
		ret = clock_control_tc3xx_ccu_pll_power(cfg->base + SCU_SYSPLLCON0,
							cfg->base + SCU_SYSPLLSTAT, true);
		ret |= clock_control_tc3xx_ccu_pll_power(cfg->base + SCU_PERPLLCON0,
							 cfg->base + SCU_PERPLLSTAT, true);
		if (ret) {
			return ret;
		}

		/* Configure pll parameters*/
		clock_control_tc3xx_ccu_pll_init(cfg->base + SCU_SYSPLLCON0, cfg->sys_p_div - 1,
						 cfg->sys_n_div - 1, 0, cfg->insel);
		clock_control_tc3xx_ccu_pll_init(cfg->base + SCU_PERPLLCON0, cfg->peri_p_div - 1,
						 cfg->peri_n_div - 1, cfg->peri_divby, 0);

		/* Set k diver values */
		clock_control_tc3xx_ccu_pll_devider(cfg->base + SCU_SYSPLLCON1,
						    cfg->base + SCU_SYSPLLSTAT, cfg->sys_k2_div - 1 + 3,
						    0, false);
		clock_control_tc3xx_ccu_pll_devider(cfg->base + SCU_PERPLLCON1,
						    cfg->base + SCU_PERPLLSTAT, cfg->peri_k2_div - 1,
						    cfg->peri_k3_div - 1, true);

		ret = clock_control_tc3xx_ccu_pll_wait_lock(cfg->base + SCU_SYSPLLSTAT,
							    cfg->base + SCU_PERPLLSTAT);
		if (ret) {
			return ret;
		}
	} else {
		/* Disable pll power */
		ret = clock_control_tc3xx_ccu_pll_power(cfg->base + SCU_SYSPLLCON0,
							cfg->base + SCU_SYSPLLSTAT, false);
		ret |= clock_control_tc3xx_ccu_pll_power(cfg->base + SCU_PERPLLCON0,
							 cfg->base + SCU_PERPLLSTAT, false);
		if (ret) {
			return ret;
		}
	}

	clock_control_tc3xx_ccu_set_divider(cfg->base);

	/* Select clock for operation */
	ccucon = cfg->base + SCU_CCUCON0;
	WAIT_FOR((*ccucon & BIT(SCU_CCUCON_LCK_POS)) == 0, 100, k_busy_wait(1));
	if (*ccucon & BIT(SCU_CCUCON_LCK_POS)) {
		return -ETIMEDOUT;
	}
	*ccucon = (*ccucon & ~SCU_CCUCON0_CLKSEL_MSK) | BIT(SCU_CCUCON_UP_POS) |
		  (cfg->clksel << SCU_CCUCON0_CLKSEL_POS);
	/* Wait for new clock config to be active */
	WAIT_FOR((*ccucon & BIT(SCU_CCUCON_LCK_POS)) == 0, 100, k_busy_wait(1));
	if (*ccucon & BIT(SCU_CCUCON_LCK_POS)) {
		return -ETIMEDOUT;
	} 

	if (cfg->clksel) {
		/* Frequency stepping, to avoid load jumps */
		int32_t i;
		for (i = 2; i >= 0; i--) {
			clock_control_tc3xx_ccu_pll_devider(cfg->base + SCU_SYSPLLCON1,
							    cfg->base + SCU_SYSPLLSTAT,
							    cfg->sys_k2_div - 1 + i, 0, false);
			/* Wait for freq to be settled, scale it due to stm div mis match */
			k_busy_wait(100 / (i+1));
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

struct clock_control_tc3xx_ccu_data {
};

#define USE_OSC_CHECK(x) DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(x)), DT_NODELABEL(fosc))
#define USE_OSC_GEN                                                                                \
	USE_OSC_CHECK(sys_pll) || USE_OSC_CHECK(fgeth) ||                                          \
		USE_OSC_CHECK(fadas) && USE_OSC_CHECK(fmcanh) || USE_OSC_CHECK(fmcan) ||           \
		USE_OSC_CHECK(fasclins)

static const struct clock_control_tc3xx_ccu_config clock_control_tc3xx_ccu_config = {
	.base = (void *)DT_INST_REG_ADDR(0),
	.use_osc = USE_OSC_GEN,
	.peri_divby = DT_PROP(DT_NODELABEL(peri_pll), divby),
	.insel = DT_NODE_CHILD_IDX(DT_CLOCKS_CTLR(DT_NODELABEL(sys_pll))),
	.clksel = DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(fsource0)), DT_NODELABEL(fpll0)),
	.sys_n_div = DT_PROP(DT_NODELABEL(sys_pll), n_div),
	.sys_p_div = DT_PROP(DT_NODELABEL(sys_pll), p_div),
	.sys_k2_div = DT_PROP(DT_NODELABEL(fpll0), clock_div),
	.peri_n_div = DT_PROP(DT_NODELABEL(peri_pll), n_div),
	.peri_p_div = DT_PROP(DT_NODELABEL(peri_pll), p_div),
	.peri_k2_div = DT_PROP(DT_NODELABEL(fpll1), clock_div),
	.peri_k3_div = DT_PROP(DT_NODELABEL(fpll2), clock_div),
};

static struct clock_control_tc3xx_ccu_data clock_control_tc3xx_ccu_data = {};

DEVICE_DT_INST_DEFINE(0, &clock_control_tc3xx_ccu_init, NULL, &clock_control_tc3xx_ccu_data,
		      &clock_control_tc3xx_ccu_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_tc3xx_ccu_api);
