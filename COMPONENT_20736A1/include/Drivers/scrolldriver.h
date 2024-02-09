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
* Define functions to access Scroll driver that uses the Quadrature hardware
*/
#ifndef __SCROLL_DRIVER_H__
#define __SCROLL_DRIVER_H__

#include "types.h"


/** \addtogroup  Scrolldriver
* \ingroup HardwareDrivers
*/
/*! @{ */
/**
* Defines a scroll driver that uses the quadrature HW.
*/


/// Turn off scroll HW. This is used when entering
/// power-off mode (software or low battery).
void scroll_turnOff(void);

/// Turn on scroll HW. This is used when exiting
/// power-off mode (software or low battery).
void scroll_turnOn(void);

/// Register a thread context interrupt handler.
/// \param userfn Pointer to the callback function to be invoked.
/// \param userdata Pointer to a context that is passed into callback when the interrupt occurs.
void scroll_registerForInterrupt(void (*userfn)(void*), void* userdata);

/// Gets the current count of the scroll wheel.
/// \return scroll count.
INT16 scroll_getCount(void);

// Initialize the Scroll driver based on the configuration.
// Will also initialize the HW if the last reset was because of a
// power up condition
void scroll_init(void);

/* @}  */
#endif
