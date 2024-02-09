/*
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
*    File Name: hw_timer.h
*
*    Abstract:  Prvide an accurate 1-shot timer (HW timer2) to app.
*
*    $History:
*           Created   02/07/14
*
*********************************************************************/

//===================================================================
//      Include
//===================================================================
#include <types.h>

/****************************************************************************
 * Use an accurate HW timer - timer2 as a 1-shot timer
 NOTE!!! NOT suggested for application use. It does not have any critical section protection.
               Use it only if your application must need an accurate timer (i.e. no way to work around).
               You need to code the callback func carefully to avoid any race condition/deadlock.
 ****************************************************************************/
typedef void (*HW_TIMER_EXPIRED_CALLBACK_FN)(void);


//===================================================================
// Functions : void hw_timer_register_timer_expired_callback
//
// The function SHOULD be called in the Application to register the timeout callback function.
// param: callback function
// param: userdata (can be class instance)
//===================================================================
void hw_timer_register_timer_expired_callback(HW_TIMER_EXPIRED_CALLBACK_FN callback);
//===================================================================
// Functions : hw_timer_start
//
// The function SHOULD be called in the Application to start the 1-shot timer. Make sure the app already
// registered the callback function.
// param: timer period in micro seconds.
//===================================================================
void hw_timer_start(UINT32 microseconds);
//===================================================================
// Functions : hw_timer_stop
//
// The function should be called in the Application to abort the 1-shot timer, in case the app does not want
// to wait for the timer to timeout
//===================================================================
void hw_timer_stop(void);

void hw_timer_int_handler(void);
