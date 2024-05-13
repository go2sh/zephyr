/*
 * Copyright 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ETH_DWC_ETH_QOS_PLATFORM_H__
#define ZEPHYR_INCLUDE_DRIVERS_ETH_DWC_ETH_QOS_PLATFORM_H__

#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

enum eth_dwc_eth_qos_clocks {
	ETH_DWC_ETH_QOS_CLK_CSR,
	ETH_DWC_ETH_QOS_CLK_APP,
	ETH_DWC_ETH_QOS_CLK_PTP
};

struct eth_dwc_eth_qos_platform_api {
	int (*platform_reset)(const struct device *dev);
	int (*clock_control_on)(const struct device *dev, enum eth_dwc_eth_qos_clocks clock);
	int (*clock_control_get_rate)(const struct device *dev, enum eth_dwc_eth_qos_clocks clock,
				      uint32_t *rate);
	int (*pinctrl_apply_state)(const struct device *dev, const struct pinctrl_dev_config *pins,
				   uint8_t id);
};

inline int eth_dwc_eth_qos_platform_reset(const struct device *dev)
{
	const struct eth_dwc_eth_qos_platform_api *api = dev->api;

	return api->platform_reset(dev);
}

inline int eth_dwc_eth_qos_clock_control_on(const struct device *dev,
					    enum eth_dwc_eth_qos_clocks clock)
{
	const struct eth_dwc_eth_qos_platform_api *api = dev->api;

	return api->clock_control_on(dev, clock);
}

inline int eth_dwc_eth_qos_clock_control_get_rate(const struct device *dev,
						  enum eth_dwc_eth_qos_clocks clock, uint32_t *rate)
{
	const struct eth_dwc_eth_qos_platform_api *api = dev->api;

	return api->clock_control_get_rate(dev, clock, rate);
}

inline int eth_dwc_eth_qos_pinctrl_apply_state(const struct device *dev,
					       const struct pinctrl_dev_config *pins, uint8_t id)
{
	const struct eth_dwc_eth_qos_platform_api *api = dev->api;

	return api->pinctrl_apply_state(dev, pins, id);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_ETH_DWC_ETH_QOS_PLATFORM_H__ */
