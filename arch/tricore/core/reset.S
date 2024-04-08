/*
 * Copyright (c) 2024 Infineon Technologies AG
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/arch/cpu.h>

GTEXT(__start)
GTEXT(z_prep_c)
GDATA(z_interrupt_stacks)
GDATA(z_triore_csa)
GDATA(__biv)
GDATA(__btv)

SECTION_FUNC(reset,__start)
    /* Load ISR and main stack pointer */
    mov     %d15, lo:(z_interrupt_stacks + CONFIG_ISR_STACK_SIZE)
    addih   %d15, %d15, hi:(z_interrupt_stacks + CONFIG_ISR_STACK_SIZE)
    mov.a   %sp, %d15
    mtcr    0xFE28, %d15

    /* Jump to cached address */
    movh.a  %a15, hi:__start_stage2
    lea     %a15,[%a15], lo:__start_stage2
    ji      %a15

SECTION_FUNC(reset,__start_stage2)
    /* Initialize context save areas */
    movh.a  %a15, hi:z_tricore_csa
    lea     %a15, [%a15], lo:z_tricore_csa
    lea     %a14, [%a15], CONFIG_TRICORE_CSA_COUNT*64
    lea     %a13, 64
    
    /* Setup FCX register */
    mov.d   %d14, %a15
    extr.u  %d13, %d14, 12, 20
    extr.u  %d12, %d14, 6, 16
    insert  %d13, %d13, %d12, 0, 16
    mtcr    0xFE38,%d13

csa_loop:
    addi    %d14, %d14, 64
    extr.u  %d13, %d14, 12, 20
    extr.u  %d12, %d14, 6, 16
    insert  %d13, %d13, %d12, 0, 16
    st.w    [%a15], %d13
    add.a   %a15, %a13
    jne.a   %a15, %a14, csa_loop

    /* Store LCX with 3 less entries for context depletion trap */
    addi    %d13, %d13, -3
    mtcr    0xFE3C,%d13

    /* Load trap and interrupt vectors */
    mov     %d15, lo:__biv
    addih   %d15, %d15, hi:__biv
    mtcr    0xFE20, %d15
    
    mov     %d15, lo:__btv
    addih   %d15, %d15, hi:__btv
    mtcr    0xFE24, %d15

    /* Call init code */
    call z_prep_c