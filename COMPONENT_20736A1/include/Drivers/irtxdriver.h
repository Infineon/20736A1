/*******************************************************************
*
* Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*
********************************************************************
*/


/** @file
*
* Define functions to access IrTx peripheral
*/
#ifndef __IR_TX_DRIVER_H__
#define __IR_TX_DRIVER_H__

#include "types.h"
#include "gpiodriver.h"

/**  \addtogroup irTx
 *  \ingroup HardwareDrivers
*/

/*! @{ */
/**
* The 20732 family includes an IRTX Hw block. The IrTx HW
* can be used to transmit IR Data. This Class defines
* an IrTx driver that can be used by applications such as
* a Remote Controller app or other related applications
* to transmit IR commands to the Host.
*
* To use the IRTX driver:
*
*
*  (1) Configure the IrTx Port and Pin to be used
*
*  example:
*   BYTE port = 0;
*   BYTE pin  = 4;
*   irTxDriver->setIrTxPortPin(port, pin);
*
*  (3) Transmit an IR Command using the senData function
*
*  example:
*  irTxDriver->sendData(testCmdArray , pktLen, irtxClockSet);
*
*  Please refer to function description for more details.
*
*/

/// Internal clock references for the IR driver.
typedef struct
{
    UINT32  modulateFreq;
    UINT8  clockSrcFreq;
    UINT8  clockSrc;
    BOOL8   invertOutput;
}IR_TX_CLOCK_SETTING;

/// IrTx HW Fifo Size.
enum
{
    IR_TX_HW_FIFO_SIZE  = 8,
};

/// Mia IR Control masks.
enum
{
    HW_MIA_IR_CTL_INIT_ENTRY_INDEX_MASK             = 0x0700,
    HW_MIA_IR_CTL_INIT_ENTRY_INDEX_SHIFT            = 8,

    HW_MIA_IR_CTL_RAW_DATA_MASK                     = 0x0040,
    HW_MIA_IR_CTL_RAW_DATA_SHIFT                    = 6,

    HW_MIA_IR_CTL_CARRIER_MODULATE_MASK             = 0x0020,
    HW_MIA_IR_CTL_CARRIER_MODULATE_ENABLE           = 0x0020,
    HW_MIA_IR_CTL_CARRIER_MODULATE_DISABLE          = 0x0000,
    HW_MIA_IR_CTL_CARRIER_MODULATE_SHIFT            = 5,

    HW_MIA_IR_CTL_MODULATE_CLK_SRC_MASK             = 0x0010,
    HW_MIA_IR_CTL_MODULATE_CLK_SRC_ACLK0            = 0x0000,
    HW_MIA_IR_CTL_MODULATE_CLK_SRC_ACLK1            = 0x0010,
    HW_MIA_IR_CTL_MODULATE_CLK_SRC_SHIFT            = 4,

    HW_MIA_IR_CTL_INVERT_OUTPUT_MASK                = 0x0008,
    HW_MIA_IR_CTL_INVERT_OUTPUT_ENABLE              = 0x0008,
    HW_MIA_IR_CTL_INVERT_OUTPUT_DISABLE             = 0x0000,
    HW_MIA_IR_CTL_INVERT_OUTPUT_SHIFT               = 3,

    HW_MIA_IR_CTL_RAW_BIT_SRC_FINAL_MASK            = 0x0004,
    HW_MIA_IR_CTL_RAW_BIT_SRC_FINAL_CMD_BIT_15      = 0x0004,
    HW_MIA_IR_CTL_RAW_BIT_SRC_PRE_BIT               = 0x0000,
    HW_MIA_IR_CTL_RAW_BIT_SRC_FINAL_SHIFT           = 2,

    HW_MIA_IR_CTL_RAW_BIT_SRC_PRE_MASK              = 0x0002,
    HW_MIA_IR_CTL_RAW_BIT_SRC_PRE_PUART_TXD         = 0x0002,
    HW_MIA_IR_CTL_RAW_BIT_SRC_PRE_IR_CTL_6          = 0x0000,
    HW_MIA_IR_CTL_RAW_BIT_SRC_PRE_SHIFT             = 1,

    HW_MIA_IR_CTL_RESET_MASK                        = 0x0001,
    HW_MIA_IR_CTL_RESET_ACTIVE                      = 0x0001,
    HW_MIA_IR_CTL_RESET_INACTIVE                    = 0x0000,
    HW_MIA_IR_CTL_RESET_SHIFT                       = 0,

    HW_MIA_IR_BUF_CTL_WRITE_ENABLE_ALL_MASK         = 0x00FF0000,
    HW_MIA_IR_BUF_CTL_WRITE_ENABLE_ENTRY_0_SHIFT    = 16,
    HW_MIA_IR_BUF_CTL_INT_ENABLE_ALL_MASK           = 0x0000FF00,
    HW_MIA_IR_BUF_CTL_INT_ENABLE_ENTRY_0_SHIFT      = 8,
    HW_MIA_IR_BUF_CTL_READY_BIT_ALL_MASK            = 0x000000FF,
    HW_MIA_IR_BUF_CTL_READY_BIT_ENTRY_0_SHIFT       = 0,

    HW_MIA_IR_CMD_IR_WAIT_TIME_MASK                 = 0x7FFF,
    HW_MIA_IR_CMD_IR_RAW_BIT_MASK                   = 0x8000,

    HW_MIA_IR_CMD0_FIFO_INDEX_MASK                  = 0x70000,
    HW_MIA_IR_CMD0_FIFO_INDEX_SHIFT                 = 16,

    HW_MIA_IR_INT_STATUS_ALL_MASK                   = 0x00FF,
    HW_MIA_IR_INT_STATUS_ENTRY_0_SHIFT              = 0,
};

/// IR Tx Status.
typedef enum IR_TX_STATUS_TAG
{
    IR_TRANSMIT_FAIL,
    IR_TRANSMIT_SUCCESS
}   IR_TX_STATUS;

/// IrTx Statemachine variables.
typedef enum IR_TX_STATE_TAG
{
    IR_TX_IDLE,
    IR_TX_PAYLOAD,
    IR_TX_BUSY,
    IR_TX_DONE,
}IR_TX_STATE;

/// IR TX initialization routine.
void irtx_IrTx(void);

/// Interrupt context TX done handler. Internal.
void  irtx_doneInterruptHandler(void);

/// IR TX done helper function. Internal.
void  irtx_doneHandler(void);

/// Update the buffer control registers. Internal.
void  irtx_updateBufferControl(UINT32 readyUpdate);

/// Configures the IR TX HW block using the ACLK.
/// \param irtxClockSet The clock setting.
void  irtx_configure(IR_TX_CLOCK_SETTING irtxClockSet);

/// Runs the internal state machine. Internal.
void  irtx_processTxData(void);

/// Sets up the GPIO port nad ppin to use for IR TX.
void    irtx_setIrTxPortPin(BYTE port, BYTE pin);

/// Transmits an array of IR tx entries.
/// \param sendBuff Pointer to an array of IR tx entries to add to tx hw fifo.
///                       The Application needs to make sure that this array does not get
///                       over written or corrupted while the transmit is still
///                       going on.
/// \param buffCount The number of entries in array.
/// \param irtxClockSet The structure to hold the clock settings.
///                         - clockSrc
///                         - clockSrcFreq
///                         - invertOutput
///                         - modulateFreq
/// \return Pass/fail indication
IR_TX_STATUS   irtx_sendData(const UINT16* sendBuff, UINT32 pktLen, IR_TX_CLOCK_SETTING irtxClockSet);

/// This function checks if the IrTx HW is available for sending data.
/// \return TRUE if available, else FALSE.
BOOL8   irtx_isAvailable(void);

/// Aborts a currently ongoing IR TX and returns the HW and driver to the idle state.
void    irtx_abortCurrentTransaction(void);

/* @} */
#endif
