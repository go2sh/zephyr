/*
 * Copyright (c) 2024 Infineon Technologies AG
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <offsets_short.h>
#include <zephyr/arch/cpu.h>
#include "asm_macros.inc"


GTEXT(z_tricore_switch)
GTEXT(z_thread_mark_switched_in)
GTEXT(__z_tricore_start_thread)


SECTION_FUNC(TEXT, __z_tricore_start_thread)
	/* Extract two CSAs */
	disable
	mfcr 	%d13, 0xFE38
	csa_to_addr %d13, %a15
	ld.w	%d12, [%a15]
	csa_to_addr	%d12, %a14
	ld.w	%d15, [%a14]
	mtcr	0xFE38, %d15
	enable

	/* Store infromation in context */
	ld.w	%d15, [%a4]+_thread_offset_to_entry
	st.w	[%a15]+__z_tricore_lower_context_t_a4_OFFSET, %d15
	ld.w	%d15, [%a4]+_thread_offset_to_p1
	st.w	[%a15]+__z_tricore_lower_context_t_a5_OFFSET, %d15
	ld.w	%d15, [%a4]+_thread_offset_to_p2
	st.w	[%a15]+__z_tricore_lower_context_t_a6_OFFSET, %d15
	ld.w	%d15, [%a4]+_thread_offset_to_p3
	st.w	[%a15]+__z_tricore_lower_context_t_a7_OFFSET, %d15
	ld.w	%d15, [%a4]+_thread_offset_to_a11
	st.w	[%a15]+__z_tricore_lower_context_t_a11_OFFSET, %d15
	ld.w	%d15, [%a4]+_thread_offset_to_a10
	st.w	[%a14]+__z_tricore_upper_context_t_a10_OFFSET, %d15
	ld.w	%d15, [%a4]+_thread_offset_to_psw
	st.w	[%a14]+__z_tricore_upper_context_t_psw_OFFSET, %d15
	mov		%d15, 0
	st.w	[%a14]+__z_tricore_upper_context_t_pcxi_OFFSET, %d15

	/* Setup upper context */
	/* Set UL and IE, clear PCPN */
	mov 	%d15, 3
	insert	%d15, %d12, %d15, 20, 10
	st.w	[%a15], %d15

	/* Set lower context */
	movh	%d15,0x0010
	andn	%d15, %d13, %d15
	mtcr	0xFE00, %d15

	/* Clear CDC */
	mfcr	%d15, 0xFE04
	mov		%d14, 0x7F
	andn	%d15, %d15, %d14
	mtcr	0xFE04,	%d15
	isync

	/* Return */
	rslcx
	rfe

	
/* void z_tricore_switch(k_thread_t *switch_to, k_thread_t *switch_from) */
SECTION_FUNC(TEXT, z_tricore_switch)
	svlcx
	/* Save old context */
	mfcr	%d15, 0xFE00
	st.w	[%a5]+_thread_offset_to_saved_pcxi, %d15
	/* TODO: Check thread state to release CSAs */

	/* Restore new context */
	ld.w	%d15, [%a4]+_thread_offset_to_saved_pcxi
	jz		%d15, __z_tricore_start_thread
	mtcr	0xFE00, %d15

	/* Clear CDC to avoid trap */
	mfcr	%d15, 0xFE04
	mov		%d14, 0x7F
	andn	%d15, %d15, %d14
	mtcr	0xFE04,	%d15
	isync

	/* Restore lower and upper context */
	rslcx
	rfe
