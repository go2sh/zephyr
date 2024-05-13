/*
 * Copyright (c) 2024 Infineon Technologies AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_INFINEON_AURIX_TC3XX_PINCTRL_SOC_H_
#define ZEPHYR_SOC_INFINEON_AURIX_TC3XX_PINCTRL_SOC_H_

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/pinctrl/tc3xx-pinctrl.h>

struct tc3xx_pinctrl {
	uint32_t pin: 4;
	uint32_t port: 5;
	uint32_t alt: 3;
	uint32_t type: 4;
	uint32_t analog: 1;
	uint32_t output: 1;
	uint32_t pull_up: 1;
	uint32_t pull_down: 1;
	uint32_t open_drain: 1;
	uint32_t output_high: 1;
	uint32_t output_low: 1;
	uint32_t output_select: 1;
	uint32_t pad_driver_mode: 2;
	uint32_t pad_level_sel: 2;
};

typedef struct tc3xx_pinctrl pinctrl_soc_pin_t;

#define RFAST_DRIVE_MODE(node, pr, idx)                                                            \
	COND_CODE_1(                                                                               \
		IS_EQ(DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), drive_strength), 0),           \
		(DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), slew_rate)),                        \
		(COND_CODE_1(                                                                      \
			IS_EQ(DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), drive_strength), 1),   \
			(2), (3))))
#define FAST_DRIVE_MODE(node, pr, idx)                                                             \
	COND_CODE_1(                                                                               \
		IS_EQ(DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), drive_strength), 0),           \
		(DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), slew_rate)),                        \
		(COND_CODE_1(                                                                      \
			IS_EQ(DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), drive_strength), 1),   \
			(2), (3))))

#define SLOW_DRIVE_MODE(node, pr, idx)                                                             \
	COND_CODE_1(IS_EQ(DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), drive_strength), 1),       \
		    (DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), slew_rate)), (3))

#define PAD_DRIVE_MODE(node, pr, idx)                                                              \
	COND_CODE_1(                                                                               \
		IS_EQ(DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), pad_type), 0),                 \
		(RFAST_DRIVE_MODE(node, pr, idx)),                                                 \
		(COND_CODE_1(IS_EQ(DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), pad_type), 1),    \
			     (FAST_DRIVE_MODE(node, pr, idx)), (SLOW_DRIVE_MODE(node, pr, idx)))))

#define Z_PINCTRL_STATE_PIN_INIT(node, pr, idx)                                                    \
	{.pin = DT_PROP_BY_PHANDLE_IDX(node, pr, idx, pinmux) & 0xF,                               \
	 .port = (DT_PROP_BY_PHANDLE_IDX(node, pr, idx, pinmux) >> 4) & 0x1F,                      \
	 .alt = (DT_PROP_BY_PHANDLE_IDX(node, pr, idx, pinmux) >> 9) & 0x7,                        \
	 .type = DT_PROP_BY_PHANDLE_IDX_OR(node, pr, idx, pin_type, 0xF),                          \
	 .analog = DT_PROP_BY_PHANDLE_IDX(node, pr, idx, input_disable) &&                         \
		   !DT_PROP_BY_PHANDLE_IDX(node, pr, idx, output_enable),                          \
	 .output = DT_PROP_BY_PHANDLE_IDX(node, pr, idx, output_enable),                           \
	 .pull_up = DT_PROP_BY_PHANDLE_IDX(node, pr, idx, bias_pull_up),                           \
	 .pull_down = DT_PROP_BY_PHANDLE_IDX(node, pr, idx, bias_pull_down),                       \
	 .open_drain = DT_PROP_BY_PHANDLE_IDX(node, pr, idx, drive_open_drain),                    \
	 .output_high = DT_PROP_BY_PHANDLE_IDX(node, pr, idx, output_high),                        \
	 .output_low = DT_PROP_BY_PHANDLE_IDX(node, pr, idx, output_low),                          \
	 .output_select = DT_PROP_BY_PHANDLE_IDX(node, pr, idx, output_select),                    \
	 .pad_driver_mode = PAD_DRIVE_MODE(node, pr, idx),                                         \
	 .pad_level_sel = DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), input_level)},

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{                                                                                          \
		DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)                      \
	}

#endif /* ZEPHYR_SOC_INFINEON_AURIX_TC3XX_PINCTRL_SOC_H_ */
