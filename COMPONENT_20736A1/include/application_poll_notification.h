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

/* File Name: application_poll_notification.h
*
* Abstract:  Provides a mechanism for the application to register for an receive
*            callbacks from the FW when a transmit opportinity is coming up.
*
*******************************************************************************/
#include "types.h"
#include "cfa.h"

#ifndef _APPLICATION_POLL_NOTIFICATION_H_
#define _APPLICATION_POLL_NOTIFICATION_H_


/// Initialize the connection event notification notification mechanism. Needs to be done once in
/// the app_create function.
void blecm_connectionEventNotifiationInit(void);


/// Register for notifications from the baseband of upcoming TX opportunities.
/// \param clientCallback The callback to be invoked.
/// \param clientContext The context (the UINT32 parameter to clientCallback) that must be passed
///                                 back to the callback.
/// \param offset Number of Bluetooth slots before/after the TX opportunity to call the callback. When negative,
///                    callback will be invoked before TX and when positive, callback will be invoked after the TX.
///                    This has to be an even number of slots (multiples of 1.25mS).
/// \param defaultPeriod When not connected, period of the callback in slots. Has to be an even number
///                     of slots. Don't set this to under ~5mS (8 slots).
/// \param connHandle - the connection handle of the connection for which we need the notifications.
void blecm_connectionEventNotifiationEnable(void (*clientCallback)(void*, UINT32), UINT32 clientContext,
                                            INT16 offset, UINT16 defaultPeriod, UINT32 connHandle);


/// Stop the connection event notification.
void blecm_connectionEventNotifiationDisable(void);

#endif  // _APPLICATION_POLL_NOTIFICATION_H_
