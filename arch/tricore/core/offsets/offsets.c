/*
 * Copyright (c) 2024 Infineon Technologies AG
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <kernel_arch_data.h>
#include <gen_offset.h>

/* Kernel offsets */
#include <kernel_offsets.h>

/* struct coop member offsets */
GEN_OFFSET_SYM(_callee_saved_t, a11);
GEN_OFFSET_SYM(_callee_saved_t, pcxi);

/* Arch struct thread offsets */
GEN_OFFSET_SYM(_thread_arch_t, entry);
GEN_OFFSET_SYM(_thread_arch_t, p1);
GEN_OFFSET_SYM(_thread_arch_t, p2);
GEN_OFFSET_SYM(_thread_arch_t, p3);
GEN_OFFSET_SYM(_thread_arch_t, a10);
GEN_OFFSET_SYM(_thread_arch_t, a11);
GEN_OFFSET_SYM(_thread_arch_t, psw);

/* CSA offsets */
GEN_OFFSET_SYM(z_tricore_lower_context_t, a11);
GEN_OFFSET_SYM(z_tricore_lower_context_t, a4);
GEN_OFFSET_SYM(z_tricore_lower_context_t, a5);
GEN_OFFSET_SYM(z_tricore_lower_context_t, a6);
GEN_OFFSET_SYM(z_tricore_lower_context_t, a7);
GEN_OFFSET_SYM(z_tricore_upper_context_t, a10);
GEN_OFFSET_SYM(z_tricore_upper_context_t, psw);
GEN_OFFSET_SYM(z_tricore_upper_context_t, pcxi);

GEN_ABS_SYM_END
