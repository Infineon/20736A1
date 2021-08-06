#ifndef _STACKNVRAM_H_
#define _STACKNVRAM_H_
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
* File Name: stacknvram.h
*
* Abstract: This file provides a central place for the NVRAM access items.
*
*
* Functions:
*
*******************************************************************************/
#include "cfa.h" // Get the Core Firmware API interfaces.

#ifdef __cplusplus
extern "C" {
#endif

//The cfa interfaces uses an item number to identify the items stored in NVRAM.
//We want to use a relative scheme so that shifting numbers becomes
//less painfull.
// If the base number is not from 0, we should update this one.
#define   STACKNVRAM_FIRST_USABLE_ITEM_NUMBER    0x00

// This item has the local root keys.
#define   STACKNVRAM_LOCAL_KEYS (STACKNVRAM_FIRST_USABLE_ITEM_NUMBER+1)
// This item has the index table associated with the bonded devices.
#define   STACKNVRAM_BOND_INDEX (STACKNVRAM_LOCAL_KEYS + 1)


// Note: This one should be the last one so that the idx can grow
// to the end. We reserve 5 bonded device, defined in lesmpkeys.h
#define   STACKNVRAM_FIRST_BONDED_IDX (STACKNVRAM_BOND_INDEX + 1)

#define   VS_BLE_HOST_LIST 0x70 //0x70 is working // (STACKNVRAM_FIRST_BONDED_IDX + 1)
#define   VS_BLE_BPM_DATA (VS_BLE_HOST_LIST + 1)
#define   VS_BLE_HRM_DATA (VS_BLE_HOST_LIST + 1)
#define   VS_BLE_THER_DATA (VS_BLE_HOST_LIST + 1)
#define   VS_BLE_GEN_DATA (VS_BLE_HOST_LIST + 1)

#define   VS_BLE_WS_DATA (VS_BLE_HOST_LIST + 1)
#define   VS_BLE_GM_DATA (VS_BLE_HOST_LIST + 1)

#define   VS_BLE_BAT_DATA (VS_BLE_HOST_LIST - 1)

#define   VS_BLE_BAT_DATA1_1 (VS_BLE_HOST_LIST - 2)

#ifdef __cplusplus
}
#endif

#endif //end of #ifndef _STACKNVRAM_H_
