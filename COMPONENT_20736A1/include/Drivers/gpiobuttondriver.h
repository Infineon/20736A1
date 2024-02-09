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
*
* File Name: gpiobuttondriver.h
*
* Abstract: This file defines the GPIO based button interface.
*
*******************************************************************************/

#ifndef __GPIO_BUTTON_DRIVER_H__
#define __GPIO_BUTTON_DRIVER_H__

#include "gpiodriver.h"


/** \addtogroup HardwareDrivers*/
/*! @{ */
/**
* This file defines the gpio based button interface. Buttons are
* implemented as keys on this platform. The hardware reports the
* state of the buttons as keys. This information is translated
* by the button interface into a byte representing various
* buttons.
*/

/*******************************************************************************
* Types and Defines
*******************************************************************************/

/*******************************************************************************
* Functions
*******************************************************************************/
// Initialize the button driver
void gpiobtn_init(UINT16 gpioMask[], UINT16 gpioConfig);

/// Disables wakeup from all buttons. Clears all pending events
void gpiobtn_wakeupDisable(void);

/// Enables wakeup from all buttons. Also clears any pending events.
/// Use config passed in constructor to configure GPIOs for wakeupEnable.
void gpiobtn_wakeupEnable(void);

/// Return the current button state as a bit mapped value. 0 for a button indicates that
/// the button is not pressed, 1 indicates the button is pressed.
/// \param port The port ID to read.
/// \return a 16 bit map representing the current state of the buttons.
UINT16 gpiobtn_getCurrentState(UINT8 port);

/// Flush any queued button activity in FW or HW and returns the current
/// state of the buttons.
void gpiobtn_flush(void);

/// Register for notification of changes.
void gpiobtn_registerForInterrupt(void (*userfn)(void*), void* userdata);

/// set the debounce delay.
void gpiobtn_setDebounce(UINT32 debDelayInUs );

/// Configure each Button GPIO
/// This configuration is used only for enabling wakeup.
void gpiobtn_configureButtonGpio(UINT16 gpioButtonConfig);

/// Interrupt Call back function
void gpiobtn_buttonPressDetected(void *userdata, UINT8 portPin);

/// Interrupt Process function
void gpiobtn_processButtonDetected(UINT8 portPin);

 /* @}  */
#endif
