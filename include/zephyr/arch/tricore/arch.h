/*
 * Copyright (c) 2024 Infineon Technologies AG
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_TRICORE_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_TRICORE_ARCH_H_

#include <zephyr/arch/tricore/thread.h>
#include <zephyr/arch/tricore/irq.h>

#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/common/ffs.h>
#if defined(CONFIG_USERSPACE)
#include <zephyr/arch/tricore/syscall.h>
#endif /* CONFIG_USERSPACE */
#include <zephyr/irq.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/devicetree.h>

#define ARCH_STACK_PTR_ALIGN  4

#ifndef _ASMLANGUAGE
#include <zephyr/sys/util.h>


#ifdef __cplusplus
extern "C" {
#endif

static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	return 0;
}

static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{

}

static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
{
	return key & 1;
}


typedef int z_arch_esf_t;

extern uint32_t sys_clock_cycle_get_32(void);

static inline uint32_t arch_k_cycle_get_32(void)
{
	return sys_clock_cycle_get_32();
}

extern uint64_t sys_clock_cycle_get_64(void);

static inline uint64_t arch_k_cycle_get_64(void)
{
	return sys_clock_cycle_get_64();
}

static ALWAYS_INLINE void arch_nop(void)
{
	__asm__ volatile("nop");
}

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_TRICORE_ARCH_H_ */
