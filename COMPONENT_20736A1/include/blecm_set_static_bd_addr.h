/*
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
*    File Name: blecm_set_static_bd_addr.h
*
*    Abstract: Set and commit a static random BD_ADDR according to
*              Bluetooth spec, Volume 3, Part C, section 10.8.1 Static address.
*              NOTE: This will set the BD_ADDR to a static random address
*                 if and only if the device address in the NV is
*                 0xFFFFFFFFFFFF. Once set, subsequent calls will always
*                 fail because the address in NV will no longer be all FF.
*
********************************************************************
*/
#ifndef _BLECM_SET_STATIC_BD_ADDR_H_
#define _BLECM_SET_STATIC_BD_ADDR_H_

#include "types.h"

/// Sets the bd_addr to a static random value and optionally commits to NV.
/// \param bd_addr OUT The static random bd_addr that was currently set. Buffer
///             must be long enough to hold the 6 byte BD_ADDR. Valid only
///             if commit succeeded. For the commit to succeed, device address
///             in NV must already be 0xFFFFFFFFFFFF.
/// \return TRUE on succes, FALSE on failure.
UINT8 blecm_set_static_random_bd_addr(UINT8* bd_addr);

#endif   // _BLECM_SET_STATIC_BD_ADDR_H_
