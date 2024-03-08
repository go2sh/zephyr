/*
 * Copyright (c) 2024 Infineon Technologies AG
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include "kernel_arch_data.h"
#include <zephyr/kernel.h>
#include <ksched.h>

union z_tricore_context __kstackmem __aligned(4 * 16) z_tricore_csa[CONFIG_TRICORE_CSA_COUNT];

#ifdef CONFIG_USERSPACE
/*
 * Per-thread (TLS) variable indicating whether execution is in user mode.
 */
__thread uint8_t is_user_mode;
#endif

void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack, char *stack_ptr,
		     k_thread_entry_t entry, void *p1, void *p2, void *p3)
{
	thread->arch.entry = (uint32_t)entry;
	thread->arch.p1 = (uint32_t)p1;
	thread->arch.p2 = (uint32_t)p2;
	thread->arch.p3 = (uint32_t)p3;
	thread->arch.a10 = (uint32_t)stack_ptr;
	thread->arch.a11 = (uint32_t)z_thread_entry;
	thread->arch.psw = (uint32_t)TRICORE_INITIAL_SYSTEM_PSW;

	thread->callee_saved.pcxi = 0;

	/* our switch handle is the thread pointer itself */
	thread->switch_handle = thread;
}

#ifdef CONFIG_USERSPACE

/*
 * User space entry function
 *
 * This function is the entry point to user mode from privileged execution.
 * The conversion is one way, and threads which transition to user mode do
 * not transition back later, unless they are doing system calls.
 */
FUNC_NORETURN void arch_user_mode_enter(k_thread_entry_t user_entry, void *p1, void *p2, void *p3)
{

}

#endif /* CONFIG_USERSPACE */

#ifndef CONFIG_MULTITHREADING

FUNC_NORETURN void z_riscv_switch_to_main_no_multithreading(k_thread_entry_t main_entry, void *p1,
							    void *p2, void *p3)
{

}
#endif /* !CONFIG_MULTITHREADING */
