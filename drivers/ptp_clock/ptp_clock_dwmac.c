/*
 * Copyright 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/devicetree.h"
#include "zephyr/drivers/ethernet/eth_dwc_eth_qos_platform.h"
#include "zephyr/net/ptp_time.h"
#define DT_DRV_COMPAT snps_dwc_eth_qos_ptp_clock

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/spinlock.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/sys_io.h>

#define MAC_TIMESTAMP_CONTROL              0x0
#define MAC_SUB_SECOND_INCREMENT           0x4
#define MAC_SYSTEM_TIME_SECONDS            0x8
#define MAC_SYSTEM_TIME_NANOSECONDS        0xC
#define MAC_SYSTEM_TIME_SECONDS_UPDATE     0x10
#define MAC_SYSTEM_TIME_NANOSECONDS_UPDATE 0x14
#define MAC_TIMESTAMP_ADDEND               0x18

#define MAC_TS_CTRL_TSENA       BIT(0)
#define MAC_TS_CTRL_TSINIT      BIT(2)
#define MAC_TS_CTRL_TSUPDT      BIT(3)
#define MAC_TS_CTRL_TSADDREG    BIT(5)
#define MAC_TS_CTRL_TSCTRLSSR   BIT(9)
#define MAC_TS_CTRL_TSVER2ENA   BIT(10)
#define MAC_TS_CTRL_TSIPENA     BIT(11)
#define MAC_TS_CTRL_AV8021ASMEN BIT(28)

struct ptp_clock_dwc_eth_qos_config {
	mm_reg_t base;
	const struct device *platform;
	uint32_t resolution;
};

struct ptp_clock_dwc_eth_qos_data {
	double clock_ratio;
	double clock_ratio_adjust;
	struct k_spinlock lock;
};

static int ptp_clock_dwc_eth_qos_set(const struct device *dev, struct net_ptp_time *tm)
{
	const struct ptp_clock_dwc_eth_qos_config *cfg = dev->config;
	struct ptp_clock_dwc_eth_qos_data *data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);
	sys_write32(tm->_sec.low, cfg->base + MAC_SYSTEM_TIME_SECONDS_UPDATE);
	sys_write32(tm->nanosecond, cfg->base + MAC_SYSTEM_TIME_NANOSECONDS_UPDATE);
	sys_write32(sys_read32(cfg->base + MAC_TIMESTAMP_CONTROL) | MAC_TS_CTRL_TSINIT,
		    cfg->base + MAC_TIMESTAMP_CONTROL);
	if (!WAIT_FOR((sys_read32(cfg->base + MAC_TIMESTAMP_CONTROL) | MAC_TS_CTRL_TSINIT) == 0, 10,
		      k_busy_wait(1))) {
		return -EIO;
	}
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int ptp_clock_dwc_eth_qos_get(const struct device *dev, struct net_ptp_time *tm)
{
	const struct ptp_clock_dwc_eth_qos_config *cfg = dev->config;
	struct ptp_clock_dwc_eth_qos_data *data = dev->data;
	struct net_ptp_time tm1, tm2;
	k_spinlock_key_t key;

	/* Double read to detect nanosecond rollover */
	key = k_spin_lock(&data->lock);
	tm1.nanosecond = sys_read32(cfg->base + MAC_SYSTEM_TIME_NANOSECONDS);
	tm1._sec.low = sys_read32(cfg->base + MAC_SYSTEM_TIME_SECONDS);
	tm2.nanosecond = sys_read32(cfg->base + MAC_SYSTEM_TIME_NANOSECONDS);
	tm2._sec.low = sys_read32(cfg->base + MAC_SYSTEM_TIME_SECONDS);
	k_spin_unlock(&data->lock, key);

	if (tm1.second == tm2.second) {
		*tm = tm1;
	} else {
		*tm = tm2;
	}
	return 0;
}

static int ptp_clock_dwc_eth_qos_adjust(const struct device *dev, int increment)
{
	const struct ptp_clock_dwc_eth_qos_config *cfg = dev->config;
	struct ptp_clock_dwc_eth_qos_data *data = dev->data;
	k_spinlock_key_t key;

	int32_t seconds;
	int32_t nanosecond;

	seconds = increment / NSEC_PER_SEC;
	nanosecond = increment - (seconds * NSEC_PER_SEC);

	key = k_spin_lock(&data->lock);
	sys_write32(seconds, cfg->base + MAC_SYSTEM_TIME_SECONDS_UPDATE);
	sys_write32(nanosecond, cfg->base + MAC_SYSTEM_TIME_NANOSECONDS_UPDATE);
	sys_write32(sys_read32(cfg->base + MAC_TIMESTAMP_CONTROL) | MAC_TS_CTRL_TSUPDT,
		    cfg->base + MAC_TIMESTAMP_CONTROL);
	if (!WAIT_FOR((sys_read32(cfg->base + MAC_TIMESTAMP_CONTROL) | MAC_TS_CTRL_TSUPDT) == 0, 10,
		      k_busy_wait(1))) {
		return -EIO;
	}
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int ptp_clock_dwc_eth_qos_rate_adjust(const struct device *dev, double ratio)
{
	const struct ptp_clock_dwc_eth_qos_config *cfg = dev->config;
	struct ptp_clock_dwc_eth_qos_data *data = dev->data;
	uint32_t addend;
	k_spinlock_key_t key;

	/* No change needed. */
	if (ratio == 1.0) {
		return 0;
	}

	ratio *= (double)data->clock_ratio_adjust;

	/* Limit possible ratio */
	if (ratio < 0.9 || ratio > 1.1) {
		return -EINVAL;
	}
	addend = UINT32_MAX * (double)data->clock_ratio * ratio;

	key = k_spin_lock(&data->lock);
	/* Save new ratio */
	data->clock_ratio_adjust = ratio;
	sys_write32(addend, cfg->base + MAC_TIMESTAMP_ADDEND);
	sys_write32(MAC_TS_CTRL_TSADDREG, cfg->base + MAC_TIMESTAMP_CONTROL);
	if (!WAIT_FOR((sys_read32(cfg->base + MAC_TIMESTAMP_CONTROL) | MAC_TS_CTRL_TSADDREG) == 0,
		      10, k_busy_wait(1))) {
		return -EIO;
	}
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int ptp_clock_dwc_eth_qos_init(const struct device *port)
{
	const struct ptp_clock_dwc_eth_qos_config *cfg = port->config;
	struct ptp_clock_dwc_eth_qos_data *data = port->data;
	int ret;
	double ptp_period, clock_period;
	uint32_t rate;
	uint32_t sub_sec_increment;

	if (!device_is_ready(cfg->platform)) {
		return -EIO;
	}

	ret = eth_dwc_eth_qos_clock_control_on(cfg->platform, ETH_DWC_ETH_QOS_CLK_PTP);
	ret |= eth_dwc_eth_qos_clock_control_get_rate(cfg->platform, ETH_DWC_ETH_QOS_CLK_PTP,
						      &rate);
	if (ret) {
		return ret;
	}

	ret = eth_dwc_eth_qos_platform_reset(cfg->platform);
	if (ret) {
		return ret;
	}

	clock_period = 1.0e9 / (double)rate;
	if (cfg->resolution) {
		ptp_period = (1.0e9 / (double)cfg->resolution);
	} else {
		ptp_period = clock_period * 2;
	}

	/* Convert ptp_period into sub second increment value */
	sub_sec_increment = (uint32_t)(ptp_period * (double)(1 << 16)) & 0xFFFF00;
	/* Store the clock ratio for addent register. Also take the error of the
	 * conversion between period and sub_sec_increment into aacount. */
	data->clock_ratio = clock_period / ((double)sub_sec_increment / (double)(1 << 16));

	sys_write32(sub_sec_increment, cfg->base + MAC_SUB_SECOND_INCREMENT);
	sys_write32((uint32_t)((double)UINT32_MAX * data->clock_ratio),
		    cfg->base + MAC_TIMESTAMP_ADDEND);
	sys_write32(MAC_TS_CTRL_TSADDREG, cfg->base + MAC_TIMESTAMP_CONTROL);
	if (!WAIT_FOR((sys_read32(cfg->base + MAC_TIMESTAMP_CONTROL) & MAC_TS_CTRL_TSADDREG) == 0,
		      10, k_busy_wait(1))) {
		return -EIO;
	}
	sys_write32(MAC_TS_CTRL_TSENA | MAC_TS_CTRL_TSCTRLSSR | MAC_TS_CTRL_TSVER2ENA |
			    MAC_TS_CTRL_TSIPENA,
		    cfg->base + MAC_TIMESTAMP_CONTROL);

	struct net_ptp_time initTime = {.second = 0, .nanosecond = 0};
	ptp_clock_dwc_eth_qos_set(port, &initTime);

	return 0;
}

static const struct ptp_clock_driver_api ptp_clock_dwc_eth_qos_api = {
	.set = ptp_clock_dwc_eth_qos_set,
	.get = ptp_clock_dwc_eth_qos_get,
	.adjust = ptp_clock_dwc_eth_qos_adjust,
	.rate_adjust = ptp_clock_dwc_eth_qos_rate_adjust,
};

#define PTP_CLOCK_DWC_ETH_QOS_INIT(n)                                                              \
	static const struct ptp_clock_dwc_eth_qos_config ptp_clock_dwc_eth_qos_##n##_config = {    \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.platform = DEVICE_DT_GET(DT_INST_PARENT(n)),                                      \
		.resolution = DT_INST_PROP_OR(n, resolution, 0),                                   \
	};                                                                                         \
                                                                                                   \
	static struct ptp_clock_dwc_eth_qos_data ptp_clock_dwc_eth_qos_##n##_data;                 \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &ptp_clock_dwc_eth_qos_init, NULL,                                \
			      &ptp_clock_dwc_eth_qos_##n##_data,                                   \
			      &ptp_clock_dwc_eth_qos_##n##_config, POST_KERNEL,                    \
			      CONFIG_PTP_CLOCK_INIT_PRIORITY, &ptp_clock_dwc_eth_qos_api);

DT_INST_FOREACH_STATUS_OKAY(PTP_CLOCK_DWC_ETH_QOS_INIT)
