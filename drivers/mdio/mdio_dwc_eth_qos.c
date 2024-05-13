/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_dwc_eth_qos_mdio, CONFIG_MDIO_LOG_LEVEL);

#define DT_DRV_COMPAT snps_dwc_eth_qos_mdio

#define PHY_OPERATION_TIMEOUT_US 250000

#define MAC_MDIO_ADDRESS      0x0
#define MAC_MDIO_ADDRESS_PA   GENMASK(25, 21)
#define MAC_MDIO_ADDRESS_RDA  GENMASK(20, 16)
#define MAC_MDIO_ADDRESS_CR   GENMASK(11, 8)
#define MAC_MDIO_ADDRESS_GOC  GENMASK(3, 2)
#define MAC_MDIO_ADDRESS_C45  BIT(1)
#define MAC_MDIO_ADDRESS_BUSY BIT(0)

#define MAC_MDIO_DATA 0x4

struct mdio_dwc_eth_qos_dev_data {
	struct k_sem sem;
};

struct mdio_dwc_eth_qos_dev_config {
	uintptr_t base_addr;
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

	k_sem_init(&dev_data->sem, 1, 1);

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
	};

#define MDIO_DWC_ETH_QOS_DEVICE(n)                                                                 \
	MDIO_DWC_ETH_QOS_CONFIG(n);                                                                \
	static struct mdio_dwc_eth_qos_dev_data mdio_dwc_eth_qos_dev_data##n;                      \
	DEVICE_DT_INST_DEFINE(n, &mdio_dwc_eth_qos_initialize, NULL,                               \
			      &mdio_dwc_eth_qos_dev_data##n, &mdio_dwc_eth_qos_dev_config_##n,     \
			      POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,                              \
			      &mdio_dwc_eth_qos_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_DWC_ETH_QOS_DEVICE)
