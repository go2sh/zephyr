/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CLOCK_CONTROL_TC3XX_PLL_H_
#define ZEPHYR_DRIVERS_CLOCK_CONTROL_TC3XX_PLL_H_

#define SCU_ID         0x08
#define SCU_OSCCON     0x10
#define SCU_SYSPLLSTAT 0x14
#define SCU_SYSPLLCON0 0x18
#define SCU_SYSPLLCON1 0x1C
#define SCU_SYSPLLCON2 0x20
#define SCU_PERPLLSTAT 0x24
#define SCU_PERPLLCON0 0x28
#define SCU_PERPLLCON1 0x2C
#define SCU_CCUCON0    0x30
#define SCU_CCUCON1    0x34
#define SCU_CCUCON2    0x40
#define SCU_CCUCON3    0x44
#define SCU_CCUCON4    0x48
#define SCU_CCUCON5    0x4C
#define SCU_CCUCON6    0x80
#define SCU_CCUCON7    0x84
#define SCU_CCUCON8    0x88
#define SCU_CCUCON9    0x8C
#define SCU_CCUCON10   0x90
#define SCU_CCUCON11   0x94

#define SCU_OSCCON_PLLLV_POS  1
#define SCU_OSCCON_MODE_POS   5
#define SCU_OSCCON_PLLHV_POS  8
#define SCU_OSCCON_OSCVAL_POS 16
#define SCU_OSCCON_MODE_MSK   (3 << SCU_OSCCON_MODE_POS)
#define SCU_OSCCON_OSCVAL_MSK (0x1F << SCU_OSCCON_OSCVAL_POS)

#define SCU_PLLSTAT_PWDSTAT_POS 1
#define SCU_PLLSTAT_LOCK_POS    2
#define SCU_PLLSTAT_K2RDY_POS   5
#define SCU_PLLSTAT_K3RDY_POS   4

#define SCU_PLLCON0_DIVBY_POS  0
#define SCU_PLLCON0_MODEN_POS  2
#define SCU_PLLCON0_PLLPWD_POS 16
#define SCU_PLLCON0_RESLD_POS  18

#define SCU_CCUCON_UP_POS  30
#define SCU_CCUCON_LCK_POS 31

#define SCU_CCUCON0_CLKSEL_POS 28
#define SCU_CCUCON0_CLKSEL_MSK (3 << SCU_CCUCON0_CLKSEL_POS)

/* Standard clock macros */
/** CCU Clock */
#define CCU_CLOCK(clk)                DT_NODELABEL(clk)
/** Get source clock for node_id */
#define CCU_SOURCE_CLOCK(node_id)     DT_CLOCKS_CTLR(node_id)
/** Get source clock for CCU_CLOCK */
#define CCU_CLOCK_SOURCE_CLOCK(clk)   DT_CLOCKS_CTLR(CCU_CLOCK(clk))
/** Check if CCU clock is queal to node */
#define CCU_CLOCK_IS_EQ(clk, node_id) IS_EQ(DT_DEP_ORD(CCU_CLOCK(clk)), DT_DEP_ORD(node_id))
/** Check if node id is a mandatory clock */
#define CCU_IS_MANDATORY(node_id)                                                                  \
	COND_CODE_1(CCU_CLOCK_IS_EQ(fsri, node_id), (1),                                           \
		    (COND_CODE_1(CCU_CLOCK_IS_EQ(fspb, node_id), (1),                              \
				 (COND_CODE_1(CCU_CLOCK_IS_EQ(ffsi, node_id), (1),                 \
					      (COND_CODE_1(CCU_CLOCK_IS_EQ(ffsi2, node_id), (1),   \
							   (0))))))))
/* Iterate over ccu clocks */
#define CCU_CLOCK_FOREACH(fn)   UTIL_CAT(DT_FOREACH_OKAY_, infineon_tc3xx_ccu_clock)(fn)
#define _CCU_CLOCK_USE_OSC(clk) DT_SAME_NODE(CCU_CLOCK_SOURCE_CLOCK(clk), CCU_CLOCK(fosc))
/** Check if fosc is used */
#define CCU_OSC_IN_USE                                                                             \
	(_CCU_CLOCK_USE_OSC(sys_pll) || _CCU_CLOCK_USE_OSC(fgeth) || _CCU_CLOCK_USE_OSC(fadas) ||  \
	 _CCU_CLOCK_USE_OSC(fmcanh) || _CCU_CLOCK_USE_OSC(fmcan) || _CCU_CLOCK_USE_OSC(fasclins))
/** Check if plls are in use */
#define CCU_USE_PLL DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(fsource0)), DT_NODELABEL(fpll0))
/** Get ccu register number for node id */
#define CCU_REG_NR(node_id)                                                                        \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, field), (DT_PROP_BY_IDX(node_id, field, 0)),         \
		    (COND_CODE_1(DT_NODE_HAS_PROP(node_id, select_field),                          \
				 (DT_PROP_BY_IDX(node_id, select_field, 0)),                       \
				 (DT_PROP_BY_IDX(node_id, cpu_field, 0)))))
/** Register address for register nr */
#define CCU_REG_ADDR(nr) (DT_INST_REG_ADDR(0) + DT_CAT(SCU_CCUCON, nr))
#define CCU_REG_UP_FLAG(node_id)                                                                   \
	COND_CODE_0(IS_EQ(CCU_REG_NR(node_id), 0), (BIT(SCU_CCUCON_UP_POS)),                       \
		    (COND_CODE_1(IS_EQ(CCU_REG_NR(node_id), 5), (BIT(SCU_CCUCON_UP_POS)), (0))))
/** Wait for ccu register nr to unlock or fail with error */
#define CCU_WAIT_FOR_UNLOCK_OR_ERR(nr)                                                             \
	if (!WAIT_FOR((sys_read32(CCU_REG_ADDR(nr)) & BIT(SCU_CCUCON_LCK_POS)) == 0, 100,          \
		      k_busy_wait(1))) {                                                           \
		return -EIO;                                                                       \
	}

/* Frquency calculation macros */
/** Geth the frequency for a node_id*/
#define CCU_FREQ(node_id)             DT_CAT(node_id, _freq)
/** Get frequency for a CCU clock*/
#define CCU_CLOCK_FREQ(clk)           CCU_FREQ(CCU_CLOCK(clk))
/** Get a fixed frequency*/
#define CCU_FIXED_CLOCK_FREQ(node_id) node_id##_freq = DT_PROP(node_id, clock_frequency)
/** Get frequency for a fixed factor clock */
#define CCU_FIXED_FACTOR_FREQ(node_id)                                                             \
	node_id##_freq = CCU_FREQ(CCU_SOURCE_CLOCK(node_id)) /                                     \
			 COND_CODE_1(DT_NODE_HAS_PROP(node_id, clock_div),                         \
				     (DT_PROP(node_id, clock_div)), (1)) *                         \
			 COND_CODE_1(DT_NODE_HAS_PROP(node_id, clock_mult),                        \
				     (DT_PROP(node_id, clock_mult)), (1))
/** Get frequency for pll clock */
#define PLL_FREQ(node_id)                                                                          \
	node_id##_freq = CCU_FREQ(CCU_SOURCE_CLOCK(node_id)) * DT_PROP(node_id, n_div) /           \
			 DT_PROP(node_id, p_div)
/** Get frequency of a device tree node_id, either fixed-factor or tc3xx pll*/
#define DT_FREQ(node_id)                                                                           \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, infineon_tc3xx_pll_clock), (PLL_FREQ(node_id)),    \
		    (CCU_FIXED_FACTOR_FREQ(node_id)))

/* Register handling*/
#define __CCU_SELECT_FIELD_VALUE(node_id)                                                          \
	COND_CODE_1(IS_EQ(DT_DEP_ORD(DT_NODELABEL(fosc)), DT_DEP_ORD(DT_PARENT(node_id))), (2),    \
		    (COND_CODE_1(IS_EQ(DT_DEP_ORD(DT_NODELABEL(fsource2)),                         \
				       DT_DEP_ORD(DT_PARENT(node_id))),                            \
				 (2), (1))))
#define _CCU_FIELD_VALUE(node_id) FIELD_PREP(_CCU_FIELD_MASK(node_id), DT_PROP(node_id, clock_div))
#define _CCU_SELECT_FIELD_VALUE(node_id)                                                           \
	FIELD_PREP(_CCU_SELECT_FIELD_MASK(node_id), __CCU_SELECT_FIELD_VALUE(node_id))
#define _CCU_CPU_FIELD_VALUE(node_id)                                                              \
	FIELD_PREP(_CCU_CPU_FIELD_MASK(node_id), 64 - DT_PROP(node_id, clock_mult))
/** Get ccu register value for node id*/
#define CCU_VALUE(node_id)                                                                         \
	(COND_CODE_1(DT_NODE_HAS_PROP(node_id, field), (_CCU_FIELD_VALUE(node_id)), (0)) |         \
	 COND_CODE_1(DT_NODE_HAS_PROP(node_id, select_field), (_CCU_SELECT_FIELD_VALUE(node_id)),  \
		     (0)) |                                                                        \
	 COND_CODE_1(DT_NODE_HAS_PROP(node_id, cpu_field), (_CCU_CPU_FIELD_VALUE(node_id)), (0)))

#define _CCU_FIELD_MASK(node_id)                                                                   \
	GENMASK(DT_PROP_BY_IDX(node_id, field, 2) + DT_PROP_BY_IDX(node_id, field, 1),             \
		DT_PROP_BY_IDX(node_id, field, 1))
#define _CCU_SELECT_FIELD_MASK(node_id)                                                            \
	GENMASK(DT_PROP_BY_IDX(node_id, select_field, 2) +                                         \
			DT_PROP_BY_IDX(node_id, select_field, 1),                                  \
		DT_PROP_BY_IDX(node_id, select_field, 1))
#define _CCU_CPU_FIELD_MASK(node_id)                                                               \
	GENMASK(DT_PROP_BY_IDX(node_id, cpu_field, 2) + DT_PROP_BY_IDX(node_id, cpu_field, 1),     \
		DT_PROP_BY_IDX(node_id, cpu_field, 1))
/* Get ccu register mask for node id*/
#define CCU_MASK(node_id)                                                                          \
	(COND_CODE_1(DT_NODE_HAS_PROP(node_id, field), (_CCU_FIELD_MASK(node_id)), (0)) |          \
	 COND_CODE_1(DT_NODE_HAS_PROP(node_id, select_field), (_CCU_SELECT_FIELD_MASK(node_id)),   \
		     (0)) |                                                                        \
	 COND_CODE_1(DT_NODE_HAS_PROP(node_id, cpu_field), (_CCU_CPU_FIELD_MASK(node_id)), (0)))

#define _CCU_REG_VALUE_SELECT(clk, reg)                                                            \
	COND_CODE_1(IS_EQ(CCU_REG_NR(clk), reg), (| CCU_VALUE(clk)), ())
/** Get ccu register for register nr */
#define CCU_REG_VALUE(nr)                                                                          \
	(0 DT_FOREACH_OKAY_VARGS_infineon_tc3xx_ccu_clock(_CCU_REG_VALUE_SELECT, nr))

#endif