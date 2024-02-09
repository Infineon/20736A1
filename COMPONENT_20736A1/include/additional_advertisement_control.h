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
*
********************************************************************

********************************************************************
*    File Name: additional_advertisement_control.h
*
*    Abstract: Provides additional advertisement control to the application.
*
*   1.   The app can choose to skip an RF activity (ADV/connection event)
*        when a configured GPIO is asserted. Useful when a very basic
*        coex is required for a beacon-like application. We don't want
*        to use this when connected. If RF activity has already started
*        before the GPIO is asserted externally, then RF activity will
*        continue normally.
*
*   2.   Notifications before and after an advertisement event.
*
********************************************************************
*/
#ifndef _ADDITIONAL_ADVERTISEMENT_CONTROL_H_
#define _ADDITIONAL_ADVERTISEMENT_CONTROL_H_

#include "types.h"

enum
{
    /// Ready to send out an adv in the next few mS. App can change ADV data if required.
    /// Typically invoked about 2.5mS before te ADV. If there are other higher prriority
    /// tasks or other events in the app thread event queue, this will be delayed.
    /// Notification is best effort.
    ADV_NOTIFICATION_READY,

    /// Just completed transmitting an ADV packet.
    ADV_NOTIFICATION_DONE
};

/// Allows the app to configure a GPIO level as an input from another system (uC/uP)
/// to indicate to the 2073x to pause RF activity for the duration the GPIO is active.
/// This can be used as a rudimentary coex mechanism. Note that this may adversely affect
/// connection stability when used incorrectly. Useful for beacon-like applications
/// colocated with WLAN.
/// \param gpio The GPIO P# that is to be used.
/// \param active_level 1 for active high, 0 for active low.
void bleprofile_configureGpioForSkippingRf(UINT8 gpio, UINT8 active_level);

/// Allows the app to register a callback that will be invoked just before an ADV is packet is about
/// to be sent out and again, immediately after.
/// \param clientCallback Pointer to a function that will be invoked in the application
///        thread context with ADV_NOTIFICATION_READY for before ADV and
///        ADV_NOTIFICATION_DONE after ADV packet is complete.
/// \param advanceNoticeInMicroseconds Number of microseconds before the ADV the notification is
///        to be sent. Will be rounded down to the nearest 1.25mS. Has to be an even multiple of 625uS.
void bleprofile_notifyAdvPacketTransmissions(void (*clientCallback)(UINT8), UINT32 advanceNoticeInMicroseconds);

/// Allows the app to modify the advertising interval lower bound
void blecm_SetAdvIntervalLowerBound(UINT8 value);

/// Allows the app to modify the default advertising non conn interval
void blecm_SetAdvNonConnIntervalDefault(UINT8 value);

#endif   // _ADDITIONAL_ADVERTISEMENT_CONTROL_H_
