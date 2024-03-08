/*
 * Copyright (c) 2024 Infineon Technologies AG
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

    	.macro csa_to_addr csa, addr
        mov     %d15, 0
        insert  %d15, %d15, \csa, 6, 16
        extr.u  %d14, \csa, 16, 4
        insert  %d15, %d15, %d14, 28, 4
        mov.a   \addr, %d15
        .endmacro