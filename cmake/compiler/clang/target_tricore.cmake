# SPDX-License-Identifier: Apache-2.0

# Required flag for preprocessing
list(APPEND NOSYSDEF_CFLAG -mcpu=${GCC_M_CPU})
# Required flag for compiler id detection
set(CMAKE_C_FLAGS_INIT -mcpu=${GCC_M_CPU})

set(TRICORE_C_FLAGS)
list(APPEND TRICORE_C_FLAGS -mcpu=${GCC_M_CPU})

list(APPEND TOOLCHAIN_C_FLAGS ${TRICORE_C_FLAGS})
list(APPEND TOOLCHAIN_LD_FLAGS NO_SPLIT ${TRICORE_C_FLAGS})
