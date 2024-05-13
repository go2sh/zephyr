/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_tc3xx_geth

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(geth_platform, CONFIG_ETHERNET_LOG_LEVEL);
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree/clocks.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/ethernet/eth_dwc_eth_qos_platform.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/net/ethernet.h>

#include "eth.h"

#define GETH_CLC     0x2000
#define GETH_GPCTL   0x2008
#define GETH_KRST0   0x2014
#define GETH_SKEWCTL 0x2040
#define DMA_MODE     0x1000
#define DMA_MODE_SWR BIT(0)

struct eth_tc3xx_geth_config {
	mm_reg_t base;
	const struct device *clkctrl;
	clock_control_subsys_t clk;
	const struct pinctrl_dev_config *eth_pins;
	const struct pinctrl_dev_config *mdio_pins;
	uint8_t interface;
};

struct eth_tc3xx_geth_data {
	bool reset_done;
};

static int eth_tc3xx_geth_reset(const struct device *dev)
{
	const struct eth_tc3xx_geth_config *cfg = dev->config;
	struct eth_tc3xx_geth_data *data = dev->data;

	if (data->reset_done) {
		return 0;
	}

	/* resets all of the MAC internal registers and logic */
	sys_write32(DMA_MODE_SWR, cfg->base + DMA_MODE);
	/* Check if reset is complete */
	if (!WAIT_FOR((sys_read32(cfg->base + DMA_MODE) & DMA_MODE_SWR) == 0, 100,
		      k_busy_wait(50))) {
		return -EIO;
	}
	data->reset_done = true;
	return 0;
}

static int eth_tc3xx_geth_clock_control_on(const struct device *dev, enum eth_dwc_eth_qos_clocks clock)
{
	const struct eth_tc3xx_geth_config *cfg = dev->config;
	switch (clock) {
	case ETH_DWC_ETH_QOS_CLK_APP:
	case ETH_DWC_ETH_QOS_CLK_PTP:
		return clock_control_on(cfg->clkctrl, cfg->clk);
	case ETH_DWC_ETH_QOS_CLK_CSR:
		return 0;
	}
	return -EINVAL;
}

static int eth_tc3xx_geth_clock_control_get_rate(const struct device *dev,
					  enum eth_dwc_eth_qos_clocks clock, uint32_t *rate)
{
	const struct eth_tc3xx_geth_config *cfg = dev->config;

	switch (clock) {
	case ETH_DWC_ETH_QOS_CLK_APP:
	case ETH_DWC_ETH_QOS_CLK_PTP:
		return clock_control_get_rate(cfg->clkctrl, cfg->clk, rate);
	case ETH_DWC_ETH_QOS_CLK_CSR:
		return clock_control_get_rate(cfg->clkctrl,
					      (void *)DT_NODE_CHILD_IDX(DT_NODELABEL(fspb)), rate);
	}
	return -EINVAL;
}

static int eth_tc3xx_geth_pinctrl_apply_state(const struct pinctrl_dev_config *pins, uint8_t id,
					      uint32_t *gpctl)
{
	uint32_t i, j;
	int ret;

	/* Apply TX pin config*/
	ret = pinctrl_apply_state(pins, 0);
	if (ret) {
		return ret;
	}

	/* Apply RX pin config */
	for (i = 0; i < pins->state_cnt; i++) {
		if (pins->states[i].id != 0) {
			continue;
		}
		for (j = 0; j < pins->states[i].pin_cnt; j++) {
			if (pins->states[i].pins[j].type > 10) {
				continue;
			}
			*gpctl = (*gpctl & ~(0x3 << pins->states[i].pins[j].type * 2)) |
				 FIELD_PREP(0x3 << pins->states[i].pins[j].type * 2,
					    pins->states[i].pins[j].alt);
		}
	}

	return 0;
}

static int eth_tc3xx_geth_init(const struct device *dev)
{
	const struct eth_tc3xx_geth_config *cfg = dev->config;
	uint32_t gpctl;
	int ret;

	if (!device_is_ready(cfg->clkctrl)) {
		return -EIO;
	}

	/* Select interface */
	gpctl = sys_read32(cfg->base + GETH_GPCTL);
	if (cfg->eth_pins) {
		eth_tc3xx_geth_pinctrl_apply_state(cfg->eth_pins, 0, &gpctl);
	}
	if (cfg->mdio_pins) {
		eth_tc3xx_geth_pinctrl_apply_state(cfg->mdio_pins, 0, &gpctl);
	}
	sys_write32(gpctl, cfg->base + GETH_GPCTL);

	ret = eth_tc3xx_geth_clock_control_on(dev, ETH_DWC_ETH_QOS_CLK_APP);
	if (ret) {
		return ret;
	}

	/* Enable Module */
	if (!aurix_enable_clock(cfg->base + GETH_CLC, 1000)) {
		return -EIO;
	}
	
	sys_write32(0, cfg->base + GETH_SKEWCTL);

	if (!aurix_kernel_reset(cfg->base + GETH_KRST0, 1000)) {
		return -EIO;
	}
	k_busy_wait(1);
	gpctl |= (cfg->interface << 22);
	sys_write32(gpctl, cfg->base + GETH_GPCTL);
	if (cfg->interface == 1) {
		sys_write32(9, cfg->base + GETH_SKEWCTL);
	}

	return 0;
}

#define ETH_TC3XX_GETH_INTERFACE(id) DT_CAT(INTERFACE_TYPE_, id)
#define INTERFACE_TYPE_0             0
#define INTERFACE_TYPE_1             4
#define INTERFACE_TYPE_3             1

struct eth_dwc_eth_qos_platform_api api = {
	.platform_reset = eth_tc3xx_geth_reset,
	.clock_control_get_rate = eth_tc3xx_geth_clock_control_get_rate,
	.clock_control_on = eth_tc3xx_geth_clock_control_on,
};

#define ETH_TC3XX_GETH_CHECK_CON_TYPE(n)                                                           \
	COND_CODE_1(DT_NODE_HAS_COMPAT(n, snps_designware_ethernet),                               \
		    (DT_ENUM_IDX(n, phy_connection_type)), ())
#define ETH_TC3XX_GETH_GET_CON_TYPE(n)                                                             \
	DT_FOREACH_CHILD(DT_DRV_INST(n), ETH_TC3XX_GETH_CHECK_CON_TYPE)

#define __ETH_TC3XX_GETH_ETH_PINS(n)                                                               \
	COND_CODE_1(DT_NODE_HAS_COMPAT(n, snps_designware_ethernet),                               \
		    (PINCTRL_DT_DEV_CONFIG_GET(n)), ())
#define ETH_TC3XX_GETH_ETH_PINS(n) DT_FOREACH_CHILD(DT_DRV_INST(n), __ETH_TC3XX_GETH_ETH_PINS)

#define __ETH_TC3XX_GETH_MDIO_PINS(n)                                                              \
	COND_CODE_1(DT_NODE_HAS_COMPAT(n, snps_dwc_eth_qos_mdio), (PINCTRL_DT_DEV_CONFIG_GET(n)),  \
		    ())
#define ETH_TC3XX_GETH_MDIO_PINS(n) DT_FOREACH_CHILD(DT_DRV_INST(n), __ETH_TC3XX_GETH_MDIO_PINS)

#define __ETH_TC3XX_GETH_CHILD_PINS(n)                                                             \
	COND_CODE_1(DT_PINCTRL_HAS_IDX(n, 0), (PINCTRL_DT_DEFINE(n);), ())
#define ETH_TC3XX_GETH_CHILD_PINS(n) DT_FOREACH_CHILD(DT_DRV_INST(n), __ETH_TC3XX_GETH_CHILD_PINS)

#define ETH_TC3XX_GETH_INIT(n)                                                                     \
	ETH_TC3XX_GETH_CHILD_PINS(n)                                                               \
	static const struct eth_tc3xx_geth_config geth_config##n = {                               \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clkctrl = DEVICE_DT_GET(DT_PARENT(DT_INST_CLOCKS_CTLR_BY_NAME(n, app))),          \
		.clk = (void *)DT_NODE_CHILD_IDX(DT_INST_CLOCKS_CTLR_BY_NAME(n, app)),             \
		.eth_pins = ETH_TC3XX_GETH_ETH_PINS(n),                                            \
		.mdio_pins = ETH_TC3XX_GETH_MDIO_PINS(n),                                          \
		.interface = ETH_TC3XX_GETH_INTERFACE(ETH_TC3XX_GETH_GET_CON_TYPE(n))};            \
	static struct eth_tc3xx_geth_data geth_data##n = {};                                       \
	DEVICE_DT_INST_DEFINE(n, eth_tc3xx_geth_init, NULL, &geth_data##n, &geth_config##n,        \
			      POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(ETH_TC3XX_GETH_INIT)