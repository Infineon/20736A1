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
* File Name: get_nv_info.c
*
* Abstract:  Provides some extra API for getting the type and size of NV.
*
*******************************************************************************/
#include "types.h"
#include "cfa.h"

/// Get the current type of the NV.
/// \return One of CFA_CONFIG_LOCATION_EEPROM, CFA_CONFIG_LOCATION_SERIAL_FLASH
///             or CFA_CONFIG_LOCATION_RAM_BUF
UINT8 get_nv_info_Type(void);

/// Returns the size of NV in bytes. Recognizes 16K, 32K, 64K, 128K, 256K and 1M.
/// Larger NVs will return 0xFFFFFFFF. There is one caveat to using this - it determines
/// the size heuristically based on address wraparound at the max boundary. If this will not happen
/// with this NV, then the behavior is undefined.
UINT32 get_nv_info_Size(void);
