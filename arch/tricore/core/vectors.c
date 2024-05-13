/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>

extern void _irq_wrapper();

#define TRAP_FUNC(prio, _)                                                                         \
	void __attribute((naked, weak, section(".vectors.trap"))) __trap_vector_tc0_class##prio()  \
	{                                                                                          \
		__asm volatile("   .p2align 5\n");                                                 \
		__asm volatile("   svlcx\n");                                                      \
		__asm volatile("   debug \n"                                                       \
			       "   rslcx\n"                                                        \
			       "   rfe\n");                                                        \
	}
LISTIFY(8, TRAP_FUNC, ())

#define VECTOR_FUNC(prio, _)                                                                       \
	void __attribute((naked, weak, section(".vectors.irq"))) __irq_vector_tc0_##prio()         \
	{                                                                                          \
		__asm volatile("   .p2align 5\n"                                                   \
			       "   svlcx\n"                                                        \
			       "   j _isr_wrapper\n"                                               \
			       "   rslcx\n"                                                        \
			       "   rfe\n");                                                        \
	}

LISTIFY(256, VECTOR_FUNC, ())
