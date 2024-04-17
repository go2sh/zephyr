/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/arch/common/sys_io.h"
#include "zephyr/sys/util.h"
#include <errno.h>
#include <stdint.h>
#define DT_DRV_COMPAT snps_dwc_eth_qos_mdio

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mdio_dwc_eth_qos_mdio, CONFIG_MDIO_LOG_LEVEL);

#define PHY_OPERATION_TIMEOUT_US 250000

#define MAC_MDIO_ADDRESS      0x200
#define MAC_MDIO_ADDRESS_PA   GENMASK(25, 21)
#define MAC_MDIO_ADDRESS_RDA  GENMASK(20, 16)
#define MAC_MDIO_ADDRESS_CR   GENMASK(11, 8)
#define MAC_MDIO_ADDRESS_GOC  GENMASK(3, 2)
#define MAC_MDIO_ADDRESS_C45  BIT(1)
#define MAC_MDIO_ADDRESS_BUSY BIT(0)

#define MAC_MDIO_DATA 0x204

#define GETH_CLC_OFFSET   0x2000
#define GETH_CLC_DISR_MSK BIT(0)
#define GETH_CLC_DISS_MSK BIT(1)

#define GETH_GPCTL_OFFSET 0x2008

struct mdio_dwc_eth_qos_dev_data {
	struct k_sem sem;
};

struct mdio_dwc_eth_qos_dev_config {
	uintptr_t base_addr;
	const struct pinctrl_dev_config *pcfg;
};

static int ALWAYS_INLINE dwc_eth_qos_mdio_busy(const struct device *dev)
{
	const struct mdio_dwc_eth_qos_dev_config *const cfg = dev->config;
	return (sys_read32(cfg->base_addr + MAC_MDIO_ADDRESS) & MAC_MDIO_ADDRESS_BUSY) != 0;
}

static void ALWAYS_INLINE dwc_eth_qos_mdio_transfer(const struct device *dev, uint8_t prtad,
						    uint8_t regad, bool write, bool c45)
{
	const struct mdio_dwc_eth_qos_dev_config *const cfg = dev->config;

	sys_write32(FIELD_PREP(MAC_MDIO_ADDRESS_PA, prtad) |
			    FIELD_PREP(MAC_MDIO_ADDRESS_RDA, regad) |
			    FIELD_PREP(MAC_MDIO_ADDRESS_GOC, write ? 0x1 : 0x3) |
			    (c45 ? MAC_MDIO_ADDRESS_C45 : 0) | MAC_MDIO_ADDRESS_BUSY,
		    cfg->base_addr + MAC_MDIO_ADDRESS);
}

static int mdio_dwc_eth_qos_read(const struct device *dev, uint8_t prtad, uint8_t regad,
				 uint16_t *data)
{
	const struct mdio_dwc_eth_qos_dev_config *const cfg = dev->config;
	struct mdio_dwc_eth_qos_dev_data *const dev_data = dev->data;

	k_sem_take(&dev_data->sem, K_FOREVER);

	if (dwc_eth_qos_mdio_busy(dev)) {
		k_sem_give(&dev_data->sem);
		return -ETIMEDOUT;
	}

	dwc_eth_qos_mdio_transfer(dev, prtad, regad, false, false);

	if (!WAIT_FOR(!dwc_eth_qos_mdio_busy(dev), PHY_OPERATION_TIMEOUT_US,
		      k_sleep(K_USEC(1000)))) {
		LOG_ERR("phy timeout");
		k_sem_give(&dev_data->sem);
		return -ETIMEDOUT;
	}

	*data = sys_read32(cfg->base_addr + MAC_MDIO_DATA);

	k_sem_give(&dev_data->sem);
	return 0;
}

static int mdio_dwc_eth_qos_write(const struct device *dev, uint8_t prtad, uint8_t regad,
				  uint16_t data)
{
	const struct mdio_dwc_eth_qos_dev_config *const cfg = dev->config;
	struct mdio_dwc_eth_qos_dev_data *const dev_data = dev->data;

	k_sem_take(&dev_data->sem, K_FOREVER);

	if (dwc_eth_qos_mdio_busy(dev)) {
		k_sem_give(&dev_data->sem);
		return -ETIMEDOUT;
	}

	sys_write32(data, cfg->base_addr + MAC_MDIO_DATA);
	dwc_eth_qos_mdio_transfer(dev, prtad, regad, true, false);

	if (!WAIT_FOR(!dwc_eth_qos_mdio_busy(dev), PHY_OPERATION_TIMEOUT_US,
		      k_sleep(K_USEC(1000)))) {
		LOG_ERR("phy timeout");
		k_sem_give(&dev_data->sem);
		return -ETIMEDOUT;
	}

	k_sem_give(&dev_data->sem);
	return 0;
}

static int mdio_dwc_eth_qos_read_c45(const struct device *dev, uint8_t prtad, uint8_t devad,
				     uint16_t regad, uint16_t *data)
{
	const struct mdio_dwc_eth_qos_dev_config *const cfg = dev->config;
	struct mdio_dwc_eth_qos_dev_data *const dev_data = dev->data;

	k_sem_take(&dev_data->sem, K_FOREVER);

	if (dwc_eth_qos_mdio_busy(dev)) {
		k_sem_give(&dev_data->sem);
		return -ETIMEDOUT;
	}

	sys_write32(cfg->base_addr + MAC_MDIO_DATA, (regad << 16));
	dwc_eth_qos_mdio_transfer(dev, prtad, devad, false, true);

	if (!WAIT_FOR(!dwc_eth_qos_mdio_busy(dev), PHY_OPERATION_TIMEOUT_US,
		      k_sleep(K_USEC(1000)))) {
		LOG_ERR("phy timeout");
		k_sem_give(&dev_data->sem);
		return -ETIMEDOUT;
	}
	*data = sys_read32(cfg->base_addr + MAC_MDIO_DATA) & 0xFFFF;

	k_sem_give(&dev_data->sem);
	return 0;
}

static int mdio_dwc_eth_qos_write_c45(const struct device *dev, uint8_t prtad, uint8_t devad,
				      uint16_t regad, uint16_t data)
{
	const struct mdio_dwc_eth_qos_dev_config *const cfg = dev->config;
	struct mdio_dwc_eth_qos_dev_data *const dev_data = dev->data;

	k_sem_take(&dev_data->sem, K_FOREVER);

	if (dwc_eth_qos_mdio_busy(dev)) {
		k_sem_give(&dev_data->sem);
		return -ETIMEDOUT;
	}

	sys_write32(cfg->base_addr + MAC_MDIO_DATA, (regad << 16) | data);
	dwc_eth_qos_mdio_transfer(dev, prtad, devad, true, true);

	if (!WAIT_FOR(!dwc_eth_qos_mdio_busy(dev), PHY_OPERATION_TIMEOUT_US,
		      k_sleep(K_USEC(1000)))) {
		LOG_ERR("phy timeout");
		k_sem_give(&dev_data->sem);
		return -ETIMEDOUT;
	}

	k_sem_give(&dev_data->sem);
	return 0;
}

static void mdio_dwc_eth_qos_bus_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void mdio_dwc_eth_qos_bus_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static int mdio_dwc_eth_qos_initialize(const struct device *dev)
{
	const struct mdio_dwc_eth_qos_dev_config *const cfg = dev->config;
	struct mdio_dwc_eth_qos_dev_data *const dev_data = dev->data;
	int res;

	k_sem_init(&dev_data->sem, 1, 1);

	uint32_t clc = sys_read32(cfg->base_addr + GETH_CLC_OFFSET);
	/* Enable Module */
	sys_write32(sys_read32(cfg->base_addr + GETH_CLC_OFFSET) & ~GETH_CLC_DISR_MSK,
		    cfg->base_addr + GETH_CLC_OFFSET);
	if (!WAIT_FOR((sys_read32(cfg->base_addr + GETH_CLC_OFFSET) & GETH_CLC_DISS_MSK) == 0, 1000,
		      k_busy_wait(1))) {
		return -ETIMEDOUT;
	}

	res = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (res != 0) {
		return res;
	}

	uint32_t i, j;
	uint32_t gpctl = sys_read32(cfg->base_addr + GETH_GPCTL_OFFSET);
	for (i = 0; i < cfg->pcfg->state_cnt; i++) {
		if (cfg->pcfg->states[i].id != 0) {
			continue;
		}
		for (j = 0; j < cfg->pcfg->states[i].pin_cnt; j++) {
			if (cfg->pcfg->states[i].pins[j].type > 10) {
				continue;
			}
			gpctl = (gpctl & (0x3 << cfg->pcfg->states[i].pins[j].type * 2)) |
				FIELD_PREP(0x3 << cfg->pcfg->states[i].pins[j].type * 2,
					   cfg->pcfg->states[i].pins[j].alt);
		}
	}
	sys_write32(gpctl, cfg->base_addr + GETH_GPCTL_OFFSET);

	return 0;
}

static const struct mdio_driver_api mdio_dwc_eth_qos_driver_api = {
	.read = mdio_dwc_eth_qos_read,
	.write = mdio_dwc_eth_qos_write,
	.read_c45 = mdio_dwc_eth_qos_read_c45,
	.write_c45 = mdio_dwc_eth_qos_write_c45,
	.bus_enable = mdio_dwc_eth_qos_bus_enable,
	.bus_disable = mdio_dwc_eth_qos_bus_disable,
};

#define MDIO_DWC_ETH_QOS_CONFIG(n)                                                                 \
	static const struct mdio_dwc_eth_qos_dev_config mdio_dwc_eth_qos_dev_config_##n = {        \
		.base_addr = DT_INST_REG_ADDR(n),                                                  \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
	};

#define MDIO_DWC_ETH_QOS_DEVICE(n)                                                                 \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	MDIO_DWC_ETH_QOS_CONFIG(n);                                                                \
	static struct mdio_dwc_eth_qos_dev_data mdio_dwc_eth_qos_dev_data##n;                      \
	DEVICE_DT_INST_DEFINE(n, &mdio_dwc_eth_qos_initialize, NULL,                               \
			      &mdio_dwc_eth_qos_dev_data##n, &mdio_dwc_eth_qos_dev_config_##n,     \
			      POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,                              \
			      &mdio_dwc_eth_qos_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_DWC_ETH_QOS_DEVICE)
