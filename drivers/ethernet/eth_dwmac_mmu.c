/*
 * Driver for Synopsys DesignWare MAC
 *
 * Copyright (c) 2021 BayLibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME dwmac_plat
#define LOG_LEVEL       CONFIG_ETHERNET_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define DT_DRV_COMPAT snps_designware_ethernet

#include <sys/types.h>
#include <zephyr/net/ethernet.h>
#include "eth_dwmac_priv.h"

#define ETH_DWMAC_COMMON_INIT(n)                                                                   \
	DWMAC_DEVICE(n)                                                                            \
	static struct dwmac_priv dwmac_instance##n = {};                                           \
	static struct dwmac_config dwmac_config##n = DWMAC_DT_INST_CONFIG(n);                      \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, dwmac_probe, NULL, &dwmac_instance##n, &dwmac_config##n,  \
				      CONFIG_ETH_INIT_PRIORITY, &dwmac_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_DWMAC_GETH_INIT)