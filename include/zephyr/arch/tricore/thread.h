/*
 * Copyright (c) 2024 Infineon Technologies AG
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_TRICORE_THREAD_H_
#define ZEPHYR_INCLUDE_ARCH_TRICORE_THREAD_H_

#ifndef _ASMLANGUAGE
#include <zephyr/types.h>


#define TRICORE_INITIAL_SYSTEM_PSW \
    ( 0x000008FFUL ) /* Supervisor Mode, MPU Register Set 0 and Call Depth Counting disabled. */

struct _callee_saved {
	uint32_t pcxi;
	uint32_t a11;
};

typedef struct _callee_saved _callee_saved_t;

struct _thread_arch {
	uint32_t entry;
	uint32_t p1;
	uint32_t p2;
	uint32_t p3;
	uint32_t a10;
	uint32_t a11;
	uint32_t psw;
};

typedef struct _thread_arch _thread_arch_t;

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_TREAD_THREAD_H_ */
