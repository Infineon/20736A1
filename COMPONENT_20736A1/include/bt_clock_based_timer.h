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
*
* File Name: bt_clock_based_timer.h
*
* Abstract:
* Bluetooth clock based periodic timer that can enqueue the callback in the application
* thread context. This is a more accurate timer than the software timers and is unaffected by sleep
* and wake cycles. Any jitter, if present is generally due to higher priority Bluetooth
* tasks/interrupts. The resolution of the timer is 1.25mS (one Bluetooth frame) and it is
* recommended that the period of the timer be kept at 5mS or more.  Shorter timeouts will adversely
* affect Bluetooth performance.
*
*******************************************************************************/
#include "types.h"

/// Initialize the Bluetooth clock based timer module. Has to be done once at initialization
/// before using the modile. Initializing a second time will invoke undefined behavoior.
void bt_clock_based_periodic_timer_Init(void);

/// Enable/start the periodic timer.
/// \param clientCallback The application callback function that is serialized to the
///        application thread. The function is required to return BLE_APP_EVENT_NO_ACTION if
///        the clientContext was not allocated using cfa_mm_Alloc() or some memory that
///        should not be freed. If the system should free clientContext using cfa_mm_Free,
///        return BLE_APP_EVENT_FREE_BUFFER.
/// \param clientContext Any context that is to be passed back to clientCallback when invoked.
/// \param defaultPeriod The default period in Bluetooth slots. 1 slot = 626uS and this has to
///        be an even number of slots. So, for 100ms, use 100000/625; for 12.5ms, use 12500/625.
void bt_clock_based_periodic_timer_Enable(int (*clientCallback)(void*), void* clientContext, UINT16 defaultPeriod);

/// Disable the periodic timer. Not necessary to disable before restarting.
void bt_clock_based_periodic_timer_Disable(void);
