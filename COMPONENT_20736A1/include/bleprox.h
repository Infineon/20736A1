#ifndef _BLEPROX_H_
#define _BLEPROX_H_
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
 */

/** @file
*
* LE Proximity client profile, service, application
*
* Refer to Bluetooth SIG Proximity Profile1.0 specifications for details.
*
* This file contains definitions and function declarations for the proximity
* profile.
*
*/
#include "bleprofile.h"



//////////////////////////////////////////////////////////////////////////////
//                      public data type definition.
//////////////////////////////////////////////////////////////////////////////
#ifdef _WIN32
#include <pshpack1.h>
#endif
// GHS syntax.
#pragma pack(1)


//Alert mode
enum ble_prox_alert_level
{
    NO_ALERT                        = 0,
    MILD_ALERT                      = 1,
    HIGH_ALERT                      = 2,
    UNDIRECTED_DISCOVERABLE_ALERT   = 3,
};

//host information for NVRAM
typedef PACKED struct
{
    BD_ADDR     bdAddr;             // BD address of the bonded host
    UINT16      findme_hdl;         // Handle of immediate alert characteristic used by findme logic
}  BLEPROX_HOSTINFO;

#ifdef _WIN32
#include <poppack.h>
#endif
// GHS syntax.
#pragma pack()


//////////////////////////////////////////////////////////////////////////////
//                      public interface declaration
//////////////////////////////////////////////////////////////////////////////
void   bleprox_Create(void);
UINT32 bleprox_ProxButton(UINT32 function);

extern const UINT8 bleprox_db_data[];
extern const UINT16 bleprox_db_size;
extern const BLE_PROFILE_CFG bleprox_cfg;
extern const BLE_PROFILE_PUART_CFG bleprox_puart_cfg;
extern const BLE_PROFILE_GPIO_CFG bleprox_gpio_cfg;

#endif // end of #ifndef _BLEPROX_H_
