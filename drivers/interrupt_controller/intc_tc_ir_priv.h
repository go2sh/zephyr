/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_TC_IR_PRIV_H_
#define ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_TC_IR_PRIV_H_


typedef struct SRCR_Bits
{
    uint32_t SRPN:8;            /**< \brief [7:0] Service Request Priority Number (rw) */
    uint32_t reserved_8:2;      /**< \brief [9:8] \internal Reserved */
    uint32_t SRE:1;             /**< \brief [10:10] Service Request Enable (rw) */
    uint32_t TOS:3;             /**< \brief [13:11] Type of Service Control (rw) */
    uint32_t reserved_14:2;     /**< \brief [15:14] \internal Reserved */
    uint32_t ECC:5;             /**< \brief [20:16] Error Correction Code (rwh) */
    uint32_t reserved_21:3;     /**< \brief [23:21] \internal Reserved */
    uint32_t SRR:1;             /**< \brief [24:24] Service Request Flag (rh) */
    uint32_t CLRR:1;            /**< \brief [25:25] Request Clear Bit (w) */
    uint32_t SETR:1;            /**< \brief [26:26] Request Set Bit (w) */
    uint32_t IOV:1;             /**< \brief [27:27] Interrupt Trigger Overflow Bit (rh) */
    uint32_t IOVCLR:1;          /**< \brief [28:28] Interrupt Trigger Overflow Clear Bit (w) */
    uint32_t SWS:1;             /**< \brief [29:29] SW Sticky Bit (rh) */
    uint32_t SWSCLR:1;          /**< \brief [30:30] SW Sticky Clear Bit (w) */
    uint32_t reserved_31:1;     /**< \brief [31:31] \internal Reserved */
} SRCR_Bits;

/** \}  */
/******************************************************************************/
/******************************************************************************/
/** \addtogroup IfxSfr_src_Registers_union
 * \{   */
/** \brief    */
typedef union
{
    uint32_t U;                 /**< \brief Unsigned access */
    int32_t I;                 /**< \brief Signed access */
    SRCR_Bits B;              /**< \brief Bitfield access */
} SRCR;


#define GET_SRC(irq) (volatile SRCR*)(SRC_BASE + irq*4)
#define GET_TOS(coreId) (coreId == 0 ? coreId : coreId + 1)

#endif