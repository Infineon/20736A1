/*******************************************************************
*
* Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
* This file defines the button interface provided by the 20732 core.
* Buttons are implemented as keys on the 20732. A scan matrix of 1 row, n cols
* is created and scanned by the key scan hardware. The hardware reports the
* state of the buttons as keys. This information is translated by the button
* interface into a byte representing various buttons
*/
#ifndef __BUTTON_DRIVER_H__
#define __BUTTON_DRIVER_H__

#include "keyscan.h"

// Forward declaration
/** \addtogroup  ButtonDriver
* \ingroup HardwareDrivers
*/
/*! @{ */
/**
* This file defines the button interface provided by the 20732 core.
* Buttons are implamented as keys on the 20732. A scan matrix of 1 row, n cols
* is created and scanned by the key scan hardware. The hardware reports the
* state of the buttons as keys. This information is translated by the button
* interface into a byte representing various buttons.
*/

/*******************************************************************************
* Types and Defines
*******************************************************************************/
/// Disable wakeup from all buttons. Clear all pending events.
void btn_wakeupDisable(void);

/// Enable wakeup from all buttons. Also clear any pending events.
void btn_wakeupEnable(void);

/// Return the current button state as a bit mapped value. 0 for a button indicates that
/// the button is not pressed, 1 indicates the button is pressed.
/// \return a 32 bit map representing the current state of the buttons.
UINT16 btn_getCurrentState(void);

/// Flush any queued button activity in FW or HW and returns the current
/// state of the buttons. Interpretation of the return value is the same
/// as getCurrentState().
/// \return a 32 bit map representing the current state of the buttons.
UINT16 btn_flush(void);

/// Returns whether there are any pending unprocessed button events.
/// \return TRUE if there are any pending unprocessed button events, FALSE otherwise.
BOOL32 btn_eventsPending(void);

/// Register for notification of changes.
/// Once registered, you CAN NOT UNREGISTER; registration is meant to
/// be a startup activity
/// \param userfn points to the function to call when the interrupt
/// comes and matches one of the masks (if you pass a method, it must
/// be static). The function does not need to clear the interrupt
/// status; this will be done by KeyscanDriver::lhlInterrupt(). The
/// function will be called ONCE per interrupt.
/// \param userdata will be passed back to userfn as-is; it can be used to
/// carry the "this", for example
void btn_registerForInterrupt(void (*userfn)(void*), void* userdata);

/// Initialize the button driver
void btn_init(void);

/// Process the provided button (keyscan) event.
void btn_processEvent(KeyEvent *event);

/// Button run time data.
typedef struct ButtonData
{
    /// Current state of the buttons. Note that this is in report format
    /// not the input format from the keyscan driver
    UINT16 curState;
    // Add more members to the report here, for future enhancements.
} ButtonData;

 /* @}  */
#endif
