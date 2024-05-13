/*
 * Copyright (c) 2024 Infineon Technologies AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_INFINEON_AURIX_TC3XX_SOC_H_
#define ZEPHYR_SOC_INFINEON_AURIX_TC3XX_SOC_H_

#include "zephyr/arch/common/sys_io.h"
#include "zephyr/kernel.h"
#include <stdint.h>
#include <zephyr/sys/util.h>

static inline int aurix_enable_clock(uintptr_t reg, uint32_t timeout)
{
	sys_write32(0, reg);
	return WAIT_FOR((sys_read32(reg) & 0x2) == 0, timeout, k_busy_wait(1));
}

static inline int aurix_disable_clock(uintptr_t reg, uint32_t timeout)
{
	sys_write32(1, reg);
	return WAIT_FOR((sys_read32(reg) & 0x2) == 0x2, timeout, k_busy_wait(1));
}

static inline int aurix_kernel_reset(uint32_t krst0, uint32_t timeout)
{
	sys_write32(1, krst0 + 8);
	sys_write32(1, krst0);
	sys_write32(1, krst0 + 4);
	return WAIT_FOR((sys_read32(krst0) & 0x2) == 0x2, timeout, k_busy_wait(1));
}

#endif