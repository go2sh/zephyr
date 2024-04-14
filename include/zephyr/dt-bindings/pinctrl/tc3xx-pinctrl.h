/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_TC3XX_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_TC3XX_PINCTRL_H_

/* Bit Masks */

#define TC3XX_PIN_POS  0
#define TC3XX_PIN_MASK 0xf

#define TC3XX_PORT_POS  4
#define TC3XX_PORT_MASK 0x1f

#define TC3XX_ALT_POS  9
#define TC3XX_ALT_MASK 0x7

#define TC3XX_INPUT_SELECT_A 0x0
#define TC3XX_INPUT_SELECT_B 0x1
#define TC3XX_INPUT_SELECT_C 0x2
#define TC3XX_INPUT_SELECT_D 0x3
#define TC3XX_INPUT_SELECT_E 0x4
#define TC3XX_INPUT_SELECT_F 0x5
#define TC3XX_INPUT_SELECT_G 0x6
#define TC3XX_INPUT_SELECT_H 0x7

#define TC3XX_OUTPUT_SELECT_O0 0x0
#define TC3XX_OUTPUT_SELECT_O1 0x1
#define TC3XX_OUTPUT_SELECT_O2 0x2
#define TC3XX_OUTPUT_SELECT_O3 0x3
#define TC3XX_OUTPUT_SELECT_O4 0x4
#define TC3XX_OUTPUT_SELECT_O5 0x5
#define TC3XX_OUTPUT_SELECT_O6 0x6
#define TC3XX_OUTPUT_SELECT_O7 0x7

/* Setters and getters */

#define TC3XX_INPUT_PINMUX(port, pin, alt_fun)                                                     \
	(((port) << TC3XX_PORT_POS) | ((pin) << TC3XX_PIN_POS) |                                       \
	 ((TC3XX_INPUT_SELECT_##alt_fun) << TC3XX_ALT_POS))
#define TC3XX_OUTPUT_PINMUX(port, pin, alt_fun)                                                    \
	(((port) << TC3XX_PORT_POS) | ((pin) << TC3XX_PIN_POS) |                                       \
	 ((TC3XX_OUTPUT_SELECT_##alt_fun) << TC3XX_ALT_POS))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_TC3XX_PINCTRL_H_ */
