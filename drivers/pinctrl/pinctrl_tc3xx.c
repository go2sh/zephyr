/*
 * Copyright (c) 2021 Yonatan Schachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/arch/common/sys_io.h"
#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>

#define TC3XX_OUT_OFFSET   0x0
#define TC3XX_OMR_OFFSET   0x4
#define TC3XX_IOCR_OFFSET  0x10
#define TC3XX_PDR_OFFSET   0x40
#define TC3XX_PDISC_OFFSET 0x60
#define TC3XX_PCSR_OFFSET  0x64
#define TC3XX_IN_OFFSET    0x24

#define TC3XX_IOCR_OUTPUT     0x10
#define TC3XX_IOCR_OPEN_DRAIN 0x08
#define TC3XX_IOCR_PULL_DOWN  0x01
#define TC3XX_IOCR_PULL_UP    0x02

#define PINCTRL_BASE DT_REG_ADDR_BY_IDX(DT_NODELABEL(pinctrl), 0)
#define PORT_BASE(x) PINCTRL_BASE + 0x100 * x

static void ALWAYS_INLINE atomic_ldmst_iocr(uintptr_t addr, uint32_t offset, uint32_t value)
{
	__asm("	imask %%e14, %0, %1, 5\n"
	      "	ldmst [%2]+0, %%e14\n"
	      :
	      : "d"(value), "d"(offset), "a"((uint32_t *)(addr))
	      : "e14");
}

static void ALWAYS_INLINE atomic_ldmst_pdr(uintptr_t addr, uint32_t offset, uint32_t value)
{
	__asm("	imask %%e14, %0, %1, 4\n"
	      "	ldmst [%2]+0, %%e14\n"
	      :
	      : "d"(value), "d"(offset), "a"((uint32_t *)(addr))
	      : "e14");
}

static void ALWAYS_INLINE atomic_ldmst_bit(uintptr_t addr, uint32_t offset, uint32_t value)
{
	__asm("	imask %%e14, %0, %1, 1\n"
	      "	ldmst [%2]+0, %%e14\n"
	      :
	      : "d"(value), "d"(offset), "a"((uint32_t *)(addr))
	      : "e14");
}

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t iocr;
	if (pin->analog) {
		atomic_ldmst_bit(PORT_BASE(pin->port) + TC3XX_PDISC_OFFSET, pin->pin, 1);
		return;
	}

	if (pin->output) {
		iocr = (TC3XX_IOCR_OUTPUT | (pin->open_drain << 3) | pin->alt);
		if (pin->alt == 0) {
			sys_write32((pin->output_high << pin->pin) |
					    (pin->output_low << (pin->pin + 16)),
				    PORT_BASE(pin->port) + TC3XX_OMR_OFFSET);
		}
	} else {
		iocr = (pin->pull_up << 0) | (pin->pull_down << 1);
	}
	atomic_ldmst_pdr(PORT_BASE(pin->port) + TC3XX_PDR_OFFSET + (pin->pin / 8) * 4,
			 (pin->pin & 0x7) * 4, (pin->pad_level_sel << 2) | (pin->pad_driver_mode));
	atomic_ldmst_iocr(PORT_BASE(pin->port) + TC3XX_IOCR_OFFSET + (pin->pin / 4) * 4,
			  (pin->pin & 0x3) * 8 + 3, iocr);
	atomic_ldmst_bit(PORT_BASE(pin->port) + TC3XX_PCSR_OFFSET, pin->pin, pin->output_select);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(pins++);
	}

	return 0;
}
