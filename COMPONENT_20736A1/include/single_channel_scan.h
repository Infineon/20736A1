/*******************************************************************
*
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
*
********************************************************************
*
********************************************************************************
*
* File Name: single_channel_scan.h
*
* Abstract:  Provides some extra API that allow the device to allow it to scan
*				only on one channel. If this patch is included and not setup,
*				will default to scanning only on channel 37.
*			WARNING: This is to be used and enabled only for testing.
*
*******************************************************************************/
#include "types.h"
#include "cfa.h"

#ifndef _SINGLE_CHANNEL_SCAN_H_
#define _SINGLE_CHANNEL_SCAN_H_

/// The channels to scan
enum
{
	// Scan only channel 37
	SINGLE_CHANNEL_SCAN_ONLY_37 = 0,

	// Scan only channel 38
	SINGLE_CHANNEL_SCAN_ONLY_38 = 24,

	// Scan only channel 39
	SINGLE_CHANNEL_SCAN_ONLY_39 = 78,
};

/// Set the only channel to scan.
/// \param index The channel to scan - one of SINGLE_CHANNEL_SCAN_ONLY_37
///         or SINGLE_CHANNEL_SCAN_ONLY_38 or SINGLE_CHANNEL_SCAN_ONLY_39
void single_channel_scan_SetChannelToScan(UINT8 index);

#endif   // _SINGLE_CHANNEL_SCAN_H_
