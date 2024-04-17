#include "zephyr/arch/common/sys_io.h"
#include "zephyr/sys/util.h"
#include <errno.h>
#define LOG_MODULE_NAME dwmac_plat
#define LOG_LEVEL       CONFIG_ETHERNET_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* be compatible with the HAL-based driver here */
#define DT_DRV_COMPAT infineon_tc3xx_geth

#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#include "eth.h"
#include "eth_dwmac_priv.h"

#define GETH_CLC_OFFSET   0x2000
#define GETH_CLC_DISR_MSK BIT(0)
#define GETH_CLC_DISS_MSK BIT(1)

#define GETH_GPCTL_OFFSET 0x2008

struct tc3xx_geth_config {
	const struct dwmac_config dwmac_config;
	const struct device *clock;
	const struct pinctrl_dev_config *pins;
	uint32_t interface;
};

int dwmac_bus_init(const struct device *dev)
{
	const struct tc3xx_geth_config *config = dev->config;
	uint32_t i, j;
	uint32_t gpctl;

	/* Enable Module */
	sys_write32(sys_read32(config->dwmac_config.base_addr + GETH_CLC_OFFSET) &
			    ~GETH_CLC_DISR_MSK,
		    config->dwmac_config.base_addr + GETH_CLC_OFFSET);
	if (!WAIT_FOR((sys_read32(config->dwmac_config.base_addr + GETH_CLC_OFFSET) &
		       GETH_CLC_DISS_MSK) == 0,
		      1000, k_busy_wait(1))) {
		return -ETIMEDOUT;
	}

	/* Apply TX pin config*/
	pinctrl_apply_state(config->pins, 0);

	/* Apply RX pin config */
	gpctl = sys_read32(config->dwmac_config.base_addr + GETH_GPCTL_OFFSET);
	for (i = 0; i < config->pins->state_cnt; i++) {
		if (config->pins->states[i].id != 0) {
			continue;
		}
		for (j = 0; j < config->pins->states[i].pin_cnt; j++) {
			if (config->pins->states[i].pins[j].type > 10) {
				continue;
			}
			gpctl = (gpctl & ~(0x3 << config->pins->states[i].pins[j].type * 2)) |
				FIELD_PREP(0x3 << config->pins->states[i].pins[j].type * 2,
					   config->pins->states[i].pins[j].alt);
		}
	}
	/* Select interface */
	gpctl |= (config->interface << 22);
	sys_write32(gpctl, config->dwmac_config.base_addr + GETH_GPCTL_OFFSET);

	return 0;
}

#define ETH_DT_INTERFACE(id) DT_CAT(INTERFACE_TYPE_, id)
#define INTERFACE_TYPE_0 0
#define INTERFACE_TYPE_1 4
#define INTERFACE_TYPE_2 1

#define DWMAC_RANDOM_MAC_PREFIX 0x00, 0x03, 0x19

#define ETH_DWMAC_GETH_INIT(n)                                                                     \
	DWMAC_DEVICE(n)                                                                            \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct dwmac_priv dwmac_instance##n = {};                                           \
	static struct tc3xx_geth_config tc3xx_geth_config_##n = {                                  \
		.dwmac_config = DWMAC_DT_INST_CONFIG(n),                                           \
		.clock = DEVICE_DT_GET(DT_PARENT(DT_INST_CLOCKS_CTLR(n))),                         \
		.pins = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.interface = ETH_DT_INTERFACE(DT_ENUM_IDX(DT_DRV_INST(n), phy_connection_type)),       \
	};                                                                                         \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, dwmac_probe, NULL, &dwmac_instance##n,                    \
				      &tc3xx_geth_config_##n, CONFIG_ETH_INIT_PRIORITY,            \
				      &dwmac_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_DWMAC_GETH_INIT)