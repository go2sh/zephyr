/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_TOOLCHAIN_LLVM_H_
#define ZEPHYR_INCLUDE_TOOLCHAIN_LLVM_H_

#ifndef ZEPHYR_INCLUDE_TOOLCHAIN_H_
#error Please do not include toolchain-specific headers directly, use <zephyr/toolchain.h> instead
#endif

#define __no_optimization __attribute__((optnone))

#if __clang_major__ >= 10
#define __fallthrough __attribute__((fallthrough))
#endif

#define TOOLCHAIN_CLANG_VERSION \
	((__clang_major__ * 10000) + (__clang_minor__ * 100) + \
	  __clang_patchlevel__)

#define TOOLCHAIN_HAS_PRAGMA_DIAG 1

#if TOOLCHAIN_CLANG_VERSION >= 30800
#define TOOLCHAIN_HAS_C_GENERIC 1
#define TOOLCHAIN_HAS_C_AUTO_TYPE 1
#endif

#include <zephyr/toolchain/gcc.h>

#undef BUILD_ASSERT /* clear out common version */
/* C++11 has static_assert built in */
#if defined(__cplusplus) && (__cplusplus >= 201103L)
#define BUILD_ASSERT(EXPR, MSG...) static_assert(EXPR, "" MSG)
#elif !defined(__cplusplus) && defined (__clang__) && __has_extension(c_static_assert)
#define BUILD_ASSERT(EXPR, MSG...) _Static_assert(EXPR, "" MSG)
#else
#define BUILD_ASSERT(EXPR, MSG...)
#endif

#ifndef __INT8_C
#define __INT8_C(x)	x
#endif

#ifndef INT8_C
#define INT8_C(x)	__INT8_C(x)
#endif

#ifndef __UINT8_C
#define __UINT8_C(x)	x ## U
#endif

#ifndef UINT8_C
#define UINT8_C(x)	__UINT8_C(x)
#endif

#ifndef __INT16_C
#define __INT16_C(x)	x
#endif

#ifndef INT16_C
#define INT16_C(x)	__INT16_C(x)
#endif

#ifndef __UINT16_C
#define __UINT16_C(x)	x ## U
#endif

#ifndef UINT16_C
#define UINT16_C(x)	__UINT16_C(x)
#endif

#ifndef __INT32_C
#define __INT32_C(x)	x
#endif

#ifndef INT32_C
#define INT32_C(x)	__INT32_C(x)
#endif

#ifndef __UINT32_C
#define __UINT32_C(x)	x ## U
#endif

#ifndef UINT32_C
#define UINT32_C(x)	__UINT32_C(x)
#endif

#ifndef __INT64_C
#define __INT64_C(x)	x
#endif

#ifndef INT64_C
#define INT64_C(x)	__INT64_C(x)
#endif

#ifndef __UINT64_C
#define __UINT64_C(x)	x ## ULL
#endif

#ifndef UINT64_C
#define UINT64_C(x)	__UINT64_C(x)
#endif

#ifndef __INTMAX_C
#define __INTMAX_C(x)	x
#endif

#ifndef INTMAX_C
#define INTMAX_C(x)	__INTMAX_C(x)
#endif

#ifndef __UINTMAX_C
#define __UINTMAX_C(x)	x ## ULL
#endif

#ifndef UINTMAX_C
#define UINTMAX_C(x)	__UINTMAX_C(x)
#endif

#endif /* ZEPHYR_INCLUDE_TOOLCHAIN_LLVM_H_ */
