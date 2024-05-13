
/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_TC3XX_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_TC3XX_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/sys_io.h>

#define TC3XX_OUT_OFFSET  0x0
#define TC3XX_OMR_OFFSET  0x4
#define TC3XX_IOCR_OFFSET 0x10
#define TC3XX_IN_OFFSET   0x24

#define TC3XX_IOCR_OUTPUT     0x10
#define TC3XX_IOCR_OPEN_DRAIN 0x08
#define TC3XX_IOCR_PULL_DOWN  0x01
#define TC3XX_IOCR_PULL_UP    0x02

#define TC3XX_GPIO_MODE_INPUT_TRISTATE    0
#define TC3XX_GPIO_MODE_INPUT_PULL_UP     TC3XX_IOCR_PULL_UP
#define TC3XX_GPIO_MODE_INPUT_PULL_DOWN   TC3XX_IOCR_PULL_DOWN
#define TC3XX_GPIO_MODE_OUTPUT_PUSH_PULL  (TC3XX_IOCR_OUTPUT)
#define TC3XX_GPIO_MODE_OUTPUT_OPEN_DRAIN (TC3XX_IOCR_OUTPUT | TC3XX_IOCR_OPEN_DRAIN)

struct gpio_tc3xx_config {
	struct gpio_driver_config common;
	mm_reg_t base;
};

struct gpio_tc3xx_data {
	struct gpio_driver_data common;
};

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_STM32_H_ */
