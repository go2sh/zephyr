/*
 * Copyright (c) 2024 Infineon Technologies AG
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sw_isr_table.h>
#include <offsets_short.h>

GDATA(_sw_isr_table)
GDATA(_kernel)
GTEXT(_isr_wrapper)
GTEXT(__z_tricore_start_thread)

/**
 *
 * @brief Wrapper around ISRs when inserted in software ISR table
 *
 * When inserted in the vector table, _isr_wrapper() demuxes the ISR table
 * using the running interrupt number as the index, and invokes the registered
 * ISR with its corresponding argument. When returning from the ISR, it
 * determines if a context switch needs to happen from next IRQ register.
 *
 */
SECTION_FUNC(TEXT, _isr_wrapper)
    movh.a  %a15, hi:_kernel
    lea     %a15, [%a15], lo:_kernel

    /* Increment IRQ nesting count. 
     * Irq nesting can be enabled by setting IE again. */
    ld.w    %d15, [%a15]+___cpu_t_nested_OFFSET+___kernel_t_cpus_OFFSET
    add     %d15, 1
    st.w    [%a15]+___cpu_t_nested_OFFSET+___kernel_t_cpus_OFFSET, %d15

    #ifdef CONFIG_TRACING_ISR
	call sys_trace_isr_enter
    #endif

    /* Get current irq number */
    movh.a  %a2, 0xF003
    lea     %a2, [%a2], 0x7204
    ld.w    %d15, [%a2]
    extr.u  %d15, %d15, 16, 10
    
    /* Load isr table entry */
    movh.a  %a3, hi:_sw_isr_table
    lea     %a3, [%a3], lo:_sw_isr_table
    addsc.a %a3, %a3, %d15, 3
    
    /* Load arg and function ptr */
    ld.w    %d0, [%a3]
    mov.a   %a4, %d0
    ld.w    %d1, [%a3]+4
    mov.a   %a14, %d1

    /* Call ISR */
    calli %a14
    
    #ifdef CONFIG_TRACING_ISR
	call sys_trace_isr_exit
    #endif

irq_done:
    /* Check nesting count */
    ld.w    %d15, [%a15]+___cpu_t_nested_OFFSET+___kernel_t_cpus_OFFSET
    add     %d15, -1
    st.w    [%a15]+___cpu_t_nested_OFFSET+___kernel_t_cpus_OFFSET, %d15
    jnz     %d15, no_reschedule

    /* Check pending irq for rechedule */
    mfcr    %d15, 0xFE2C
    extr.u  %d15, %d15, 16, 8
    jnz      %d15, no_reschedule

check_reschedule:
    /* TODO: Check thread state to see if we need to recycle CSAs */
    ld.a    %a4, [%a15]+___cpu_t_current_OFFSET+___kernel_t_cpus_OFFSET
    mfcr    %d15, 0xFE00
    st.w    [%a4]+___thread_t_callee_saved_OFFSET+___callee_saved_t_pcxi_OFFSET, %d15
    call    z_get_next_switch_handle
    jz.a    %a2, no_reschedule

reschedule:
    /* Load context and check for context creation */
    ld.w    %d15, [%a2]+___thread_t_callee_saved_OFFSET+___callee_saved_t_pcxi_OFFSET
    mov.aa  %a4, %a2
    jz		%d15, __z_tricore_start_thread
	mtcr	0xFE00, %d15

	/* Clear CDC to avoid trap */
	mfcr	%d15, 0xFE04
	mov		%d14, 0x7F
	andn	%d15, %d15, %d14
	mtcr	0xFE04,	%d15
	isync


no_reschedule:
    rslcx
    rfe

