/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#define DT_DRV_COMPAT infineon_asclin_uart

#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
// #include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

#define ASCLIN_CLC_OFFSET         0x00
#define ASCLIN_IOCR_OFFSET        0x04
#define ASCLIN_ID_OFFSET          0x08
#define ASCLIN_TXFIFOCON_OFFSET   0x0C
#define ASCLIN_RXFIFOCON_OFFSET   0x10
#define ASCLIN_BITCON_OFFSET      0x14
#define ASCLIN_FRAMECON_OFFSET    0x18
#define ASCLIN_DATCON_OFFSET      0x1C
#define ASCLIN_BRG_OFFSET         0x20
#define ASCLIN_BRD_OFFSET         0x24
#define ASCLIN_LINCON_OFFSET      0x28
#define ASCLIN_LINBTIMER_OFFSET   0x2C
#define ASCLIN_LINHTIMER_OFFSET   0x30
#define ASCLIN_FLAGS_OFFSET       0x34
#define ASCLIN_FLAGSSET_OFFSET    0x38
#define ASCLIN_FLAGSCLEAR_OFFSET  0x3C
#define ASCLIN_FLAGSENABLE_OFFSET 0x40
#define ASCLIN_TXDATA_OFFSET      0x44
#define ASCLIN_RXDATA_OFFSET      0x48
#define ASCLIN_CSR_OFFSET         0x4C
#define ASCLIN_RXDATAD_OFFSET     0x50

#define ASCLIN_CLC_DISR_MSK BIT(0)

#define ASCLIN_FIFOCON_FILL_MSK     GENMASK(20, 16)
#define ASCLIN_FIFOCON_FLUSH_POS    0
#define ASCLIN_FIFOCON_EN_POS       1
#define ASCLIN_FIFOCON_FM_POS       4
#define ASCLIN_FIFOCON_W_POS        6
#define ASCLIN_FIFOCON_INTLEVEL_POS 8
#define ASCLIN_FIFOCON_FILL_POS     16
#define ASCLIN_FIFOCON_BUF_POS      31

#define ASCLIN_FRAMECON_STOP_POS   9
#define ASCLIN_FRAMECON_MODE_POS   16
#define ASCLIN_FRAMECON_PARITY_POS 30
#define ASCLIN_FRAMECON_ODD_POS    31

#define ASCLIN_BRG_D_POS 0
#define ASCLIN_BRG_N_POS 16

#define ASCLIN_BITCON_PRESCALER_POS    0
#define ASCLIN_BITCON_OVERSAMPLING_POS 16
#define ASCLIN_BITCON_SAMPLEPOINT_POS  24
#define ASCLIN_BITCON_SM_POS           31

#define ASCLIN_CSR_CLKSEL_MSK GENMASK(1, 0)
#define ASCLIN_CSR_CON_MSK    BIT(31)

/* device data */
struct uart_asclin_device_data {
	struct uart_config *uart_cfg; /* stores uart config from device tree*/
	struct k_spinlock lock;
};

/*
 * device config:
 * stores data that cannot be changed during run time.
 */
struct uart_asclin_device_config {
	mm_reg_t base;
	const struct pinctrl_dev_config *pcfg;
};

static int uart_asclin_poll_in(const struct device *dev, unsigned char *p_char)
{
	const struct uart_asclin_device_config *config = dev->config;
	struct uart_asclin_device_data *data = dev->data;
	int ret_val = -1;
	uint32_t status;

	/* generate fatal error if CONFIG_ASSERT is enabled. */
	__ASSERT(p_char != NULL, "p_char is null pointer!");

	/* Stop, if p_char is null pointer */
	if (p_char == NULL) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* check if received character is ready.*/
	status = sys_read32(config->base + ASCLIN_RXFIFOCON_OFFSET);
	if (status & ASCLIN_FIFOCON_FILL_MSK) {
		/* got a character */
		*p_char = sys_read32(config->base + ASCLIN_RXDATA_OFFSET);
		ret_val = 0;
	}

	k_spin_unlock(&data->lock, key);

	return ret_val;
}

static void uart_asclin_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_asclin_device_config *config = dev->config;
	struct uart_asclin_device_data *data = dev->data;
	uint32_t status;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	do {
		/* wait until uart is free to transmit.*/
		status = sys_read32(config->base + ASCLIN_TXFIFOCON_OFFSET);
	} while ((status & ASCLIN_FIFOCON_FILL_MSK) != 0);

	sys_write32(c, config->base + ASCLIN_TXDATA_OFFSET);

	k_spin_unlock(&data->lock, key);
}

static int uart_asclin_err_check(const struct device *dev)
{
	struct uart_asclin_device_data *data = dev->data;
	int err = 0;

	return err;
}

static inline int uart_asclin_set_clk(const struct uart_asclin_device_config *config, uint8_t clk)
{
	sys_write32(clk, config->base + ASCLIN_CSR_OFFSET);
	if (clk) {
		return !WAIT_FOR(
			(sys_read32(config->base + ASCLIN_CSR_OFFSET) & ASCLIN_CSR_CON_MSK) != 0,
			1000, k_busy_wait(1));
	} else {
		return !WAIT_FOR(
			(sys_read32(config->base + ASCLIN_CSR_OFFSET) & ASCLIN_CSR_CON_MSK) == 0,
			1000, k_busy_wait(1));
	}
}

static inline int uart_asclin_set_mode_init(const struct uart_asclin_device_config *config)
{
	int ret = uart_asclin_set_clk(config, 0);
	if (ret) {
		return ret;
	}
	sys_write32(0, config->base + ASCLIN_FRAMECON_OFFSET);
}

static bool uart_asclin_set_baudrate(const struct uart_asclin_device_config *config, float baudrate)
{
	uint32_t oversampling = 16;
	uint32_t samplepoint = 8;
	uint32_t medianFilter = 1;
	uint32_t prescalar = 1;

	float fOvs;
	uint32_t d = 0, n, dBest = 1, nBest = 1;
	float f;

	/* Set the PD frequency */
	uint32_t fpd = 200.0e6;
	oversampling = oversampling > 4 ? oversampling : 4;
	samplepoint = samplepoint > 1 ? samplepoint : 1;
	fOvs = baudrate * oversampling;

	int32_t m[2][2];
	float div, startDiv;
	int32_t ai;

	startDiv = div = (fOvs / fpd);

	/* initialize matrix */
	m[0][0] = m[1][1] = 1;
	m[0][1] = m[1][0] = 0;

	/* loop finding terms until denom gets too big */
	while (m[1][0] * (ai = (uint32_t)div) + m[1][1] <= 4095) {
		int32_t t;
		t = m[0][0] * ai + m[0][1];
		m[0][1] = m[0][0];
		m[0][0] = t;
		t = m[1][0] * ai + m[1][1];
		m[1][1] = m[1][0];
		m[1][0] = t;
		div = 1 / (div - (float)ai);
	}

	sys_write32((m[0][0] << ASCLIN_BRG_N_POS) | (m[1][0] << ASCLIN_BRG_D_POS),
		    config->base + ASCLIN_BRG_OFFSET);
	sys_write32(((oversampling - 1) << ASCLIN_BITCON_OVERSAMPLING_POS) |
			    ((prescalar - 1) << ASCLIN_BITCON_PRESCALER_POS) |
			    (samplepoint << ASCLIN_BITCON_SAMPLEPOINT_POS) |
			    (medianFilter << ASCLIN_BITCON_SM_POS),
		    config->base + ASCLIN_BITCON_OFFSET);

	return true;
}

static int uart_asclin_init(const struct device *dev)
{
	const struct uart_asclin_device_config *config = dev->config;
	const struct uart_asclin_device_data *data = dev->data;

	uint32_t clc = sys_read32(config->base + ASCLIN_CLC_OFFSET);
	/* Enable Module */
	sys_write32(sys_read32(config->base + ASCLIN_CLC_OFFSET) & ~ASCLIN_CLC_DISR_MSK,
		    config->base + ASCLIN_CLC_OFFSET);

	if (!WAIT_FOR((sys_read32(config->base + ASCLIN_CLC_OFFSET) & ASCLIN_CLC_DISR_MSK) == 0,
		      1000, k_busy_wait(1))) {
		return -ETIMEDOUT;
	}

	/* Pin config */
	int status = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (status < 0) {
		return status;
	}

	if (uart_asclin_set_mode_init(config)) {
		return -ETIMEDOUT;
	}
	/* Input configuration */
	// TODO: IOCR
	/* Configuration bit timing */
	uart_asclin_set_baudrate(config, data->uart_cfg->baudrate);
	/* Frame Configuration*/
	sys_write32((data->uart_cfg->stop_bits << ASCLIN_FRAMECON_STOP_POS) |
			    (1 << ASCLIN_FRAMECON_MODE_POS) |
			    ((data->uart_cfg->parity != UART_CFG_PARITY_NONE)
			     << ASCLIN_FRAMECON_PARITY_POS) |
			    ((data->uart_cfg->parity == UART_CFG_PARITY_ODD)
			     << ASCLIN_FRAMECON_ODD_POS),
		    config->base + ASCLIN_FRAMECON_OFFSET);
	/* Data configuration */
	sys_write32(data->uart_cfg->data_bits + 4, config->base + ASCLIN_DATCON_OFFSET);
	/* Fifo configuration */
	sys_write32((1 << ASCLIN_FIFOCON_EN_POS) | (0 << ASCLIN_FIFOCON_FM_POS) |
			    ((data->uart_cfg->data_bits == UART_CFG_DATA_BITS_9 ? 2 : 1)
			     << ASCLIN_FIFOCON_W_POS),
		    config->base + ASCLIN_TXFIFOCON_OFFSET);
	sys_write32((1 << ASCLIN_FIFOCON_EN_POS) | (0 << ASCLIN_FIFOCON_FM_POS) |
			    ((data->uart_cfg->data_bits == UART_CFG_DATA_BITS_9 ? 2 : 1)
			     << ASCLIN_FIFOCON_W_POS),
		    config->base + ASCLIN_RXFIFOCON_OFFSET);

	/* Enable clock again */
	if (uart_asclin_set_clk(config, 2)) {
		return -ETIMEDOUT;
	}

	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_asclin_configure(const struct device *dev, const struct uart_config *cfg)
{
	const struct uart_asclin_device_config *config = dev->config;
	struct uart_asclin_device_data *const data = dev->data;

	if (!cfg) {
		return -EINVAL;
	}

	if (cfg->stop_bits == UART_CFG_STOP_BITS_0_5 || cfg->parity == UART_CFG_PARITY_MARK ||
	    cfg->parity == UART_CFG_PARITY_SPACE) {
		return -ENOTSUP;
	}

	uart_asclin_set_mode_init(config);
	/* Configuration bit timing */
	uart_asclin_set_baudrate(config, cfg->baudrate);
	/* Frame Configuration*/
	sys_write32((cfg->stop_bits << ASCLIN_FRAMECON_STOP_POS) | (1 << ASCLIN_FRAMECON_MODE_POS) |
			    ((cfg->parity != UART_CFG_PARITY_NONE) << ASCLIN_FRAMECON_PARITY_POS) |
			    ((cfg->parity == UART_CFG_PARITY_ODD) << ASCLIN_FRAMECON_ODD_POS),
		    config->base + ASCLIN_FRAMECON_OFFSET);
	/* Data configuration */
	sys_write32(cfg->data_bits + 4, config->base + ASCLIN_DATCON_OFFSET);
	/* Fifo configuration */
	sys_write32(
		(1 << ASCLIN_FIFOCON_EN_POS) | (0 << ASCLIN_FIFOCON_FM_POS) |
			((cfg->data_bits == UART_CFG_DATA_BITS_9 ? 2 : 1) << ASCLIN_FIFOCON_W_POS),
		config->base + ASCLIN_TXFIFOCON_OFFSET);
	sys_write32(
		(1 << ASCLIN_FIFOCON_EN_POS) | (0 << ASCLIN_FIFOCON_FM_POS) |
			((cfg->data_bits == UART_CFG_DATA_BITS_9 ? 2 : 1) << ASCLIN_FIFOCON_W_POS),
		config->base + ASCLIN_RXFIFOCON_OFFSET);

	/* Enable clock again */
	if (uart_asclin_set_clk(config, 2)) {
		return -ETIMEDOUT;
	}

	*data->uart_cfg = *cfg;

	return 0;
}

static int uart_asclin_config_get(const struct device *dev, struct uart_config *cfg)
{
	const struct uart_asclin_device_data *data = dev->data;

	__ASSERT(cfg != NULL, "cfg_out is null pointer!");

	if (cfg == NULL) {
		return -EINVAL;
	}

	*cfg = *data->uart_cfg;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_asclin_driver_api = {
	.poll_in = uart_asclin_poll_in,
	.poll_out = uart_asclin_poll_out,
	.err_check = uart_asclin_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_asclin_configure,
	.config_get = uart_asclin_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_asclin_fifo_fill,
	.fifo_read = uart_asclin_fifo_read,
	.irq_tx_enable = uart_asclin_irq_tx_enable,
	.irq_tx_disable = uart_asclin_irq_tx_disable,
	.irq_tx_ready = uart_asclin_irq_tx_ready,
	.irq_tx_complete = uart_asclin_irq_tx_complete,
	.irq_rx_enable = uart_asclin_irq_rx_enable,
	.irq_rx_disable = uart_asclin_irq_rx_disable,
	.irq_rx_ready = uart_asclin_irq_rx_ready,
	.irq_is_pending = uart_asclin_irq_is_pending,
	.irq_update = uart_asclin_irq_update,
	.irq_callback_set = uart_asclin_irq_callback_set,
#endif

#ifdef CONFIG_UART_DRV_CMD
	.drv_cmd = uart_asclin_drv_cmd,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

#define UART_TC3XX_IRQ_CONFIG_FUNC(n)
#define UART_TC3XX_IRQ_CONFIG_INIT(n)

#else

#define UART_TC3XX_IRQ_CONFIG_FUNC(n)
#define UART_TC3XX_IRQ_CONFIG_INIT(n)

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define UART_TC3XX_DEVICE_INIT(n)                                                                  \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct uart_config uart_cfg_##n = {                                                 \
		.baudrate = DT_INST_PROP_OR(n, current_speed, 112000),                             \
		.parity = DT_INST_ENUM_IDX_OR(n, parity, UART_CFG_PARITY_NONE),                    \
		.stop_bits = DT_INST_ENUM_IDX_OR(n, stop_bits, UART_CFG_STOP_BITS_1),              \
		.data_bits = DT_INST_ENUM_IDX_OR(n, data_bits, UART_CFG_DATA_BITS_8),              \
		.flow_ctrl = DT_INST_PROP(n, hw_flow_control) ? UART_CFG_FLOW_CTRL_RTS_CTS         \
							      : UART_CFG_FLOW_CTRL_NONE,           \
	};                                                                                         \
	UART_TC3XX_IRQ_CONFIG_FUNC(n)                                                              \
	static struct uart_asclin_device_data uart_asclin_dev_data_##n = {.uart_cfg =              \
										  &uart_cfg_##n};  \
                                                                                                   \
	static const struct uart_asclin_device_config uart_asclin_dev_cfg_##n = {                  \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		UART_TC3XX_IRQ_CONFIG_INIT(n)};                                                    \
	DEVICE_DT_INST_DEFINE(n, uart_asclin_init, NULL, &uart_asclin_dev_data_##n,                \
			      &uart_asclin_dev_cfg_##n, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, \
			      &uart_asclin_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_TC3XX_DEVICE_INIT)
