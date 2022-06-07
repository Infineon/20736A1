/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
 */

/*******************************************************************************
*
* \file   bleapputils.h
* \brief
*
*  Utility functions for Bluetooth LE application/profile
*
*******************************************************************************/
#include "types.h"

/// Utilities function initialization. Internal, FW initializes before the app is created.
void bleapputils_init(void);

/// Disables interrupts and preemption. Use with caution; never lock out interrupts for more than ~100uS.
/// \return Returns an opaque current posture that is to be used when restoring interrupts.
UINT32 bleapputil_cpuIntDisable(void);

/// Enables/restores interrupt and preemption state.
/// \param _newPosture The value returned by bleapputil_cpuIntDisable previously.
void bleapputil_cpuIntEnable(UINT32 _newPosture);

/// This function returns the current native Bluetooth clock; this
/// counter is 28 bits, and ticks every 312.5 us and is adjusted for
/// drift through sleep, etc. To compute differences and times elapsed,
/// use bleapputils_diffNativeBtClks() or bleapputils_BtClksSince().
/// \return the counter value.
/// \return The current Bluetooth clock value.
UINT32 bleapputils_currentNativeBtClk(void);

/// This function computes the signed difference between two Bluetooth
/// clock instants. The general "garbage-in garbage-out" principle
/// applies, so clocks must be from the same piconet, must be valid when
/// they are taken, etc. Bluetooth clock is 28 bits only and cannot be relied upon when
/// timings in the hours are desired.
/// \param from is the "from" time.
/// \param to is the "to" time.
/// \return the signed difference (to - from); positive if "to" is after "from",
/// negative otherwise.
INT32 bleapputils_diffBtClks(UINT32 from, UINT32 to);

/// This function computes the time elapsed since "before", in Bluetooth
/// clocks. This functions handles rollovers. Clock resets will cause
/// a large value to be returned.
/// \param before is the previous counter value, as was returned by
/// hiddcfa_currentNativeBtClk().
/// \return the time elapsed, in Bluetooth clocks (312.5 us).
UINT32 bleapputils_BtClksSince(UINT32 before);

/// Busy-wait for the given number of uS. CAUTION: This may trip the watchdog if used for more than
/// a few milliseconds. Any busy wait of 12.5mS or longer are discouraged.
/// \param delayUs Number of microseconds to wait for.
void bleapputils_delayUs(UINT32 delayUs);

/// Busy-wait for hundreds of microseconds while petting the watchdog. CAUTION: This is guaranteed
/// to not trip the watchdog, but this will also starve the application thread.
/// \param hundredsOfUs Number of microseconds to wait in hundreds of Us.
void bleapputils_Sleep(UINT8 hundredsOfUs);

/// Change the current LPO source to use in sleep. Internal, not for application use.
void bleapputils_changeLPOSource(UINT8 pmu_clkSrc, BOOLEAN selPeripheralLPOSrc, UINT16 driftRate);

/// Calibrates the LHL LPO and returns the value. Internal, not for application use.
UINT32 bleapputils_getLhlLpoCalibrationValue(UINT32 lpoCycles);
