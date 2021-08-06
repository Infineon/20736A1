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
*
********************************************************************************
*
* File Name: runtime_stats.h
*
* Abstract:  Provides some runtime statistics. Currently:
*            Uptime - the amount of time the device has been up since power on.
*            Sleeptime - The total amount of time the device has been asleep.
*            Awaketime - The total amount of time the device has been awake.
*
*            This initializes and uses the RTC clock. So you must include the
*            rtc_api patches. The default option is to use the internal 128 KHz
*            clock. If the external 32K is available, configure that before calling
*            runtime_stats_init().
*            The units of the above measurements are in RTC ticks. So the ratio
*            of uptime and sleeptime/awaketime might be more interesting. These
*            values are only a good approximation.
*
*******************************************************************************/
#include "types.h"
#include "cfa.h"

#ifndef _RUNTIME_STATS_H_
#define _RUNTIME_STATS_H_

/// Initialize the runtime stats module. This has to be done once.
/// preferably as the first thing in the application's create function.
void runtime_stats_init(void);

/// Return the uptime of this device.
/// \return The uptime in number of RTC ticks.
UINT64 runtime_stats_get_up_time(void);

/// Returns the total amount of time the device has slept.
/// \return The number of RTC ticks the device has slept.
UINT64 runtime_stats_get_sleep_time(void);

/// Returns the amount of time the device has stayed awake.
/// \return The number of RTC ticks the device has stayed awake.
UINT64 runtime_stats_get_awake_time(void);


#endif   // _RUNTIME_STATS_H_
