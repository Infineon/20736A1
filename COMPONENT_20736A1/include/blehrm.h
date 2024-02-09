 /*
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
*/

#ifndef _BLEHRM_H_
#define _BLEHRM_H_

/** @file
*
* LE Heart Rate profile, service, application
*
* This file contains definitions and function prototypes for LE Heart
* Rate Monitor implementation.
*
* Refer to Bluetooth SIG Heart Rate Profile 1.0 abd Heart Rate Service 1.0
* specifications for details.
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

#define RR_MAX_NUM 9

//flag enum
enum blehrm_flag
{
    HRM_HEARTRATE_VALUE = 0x01, //if set, 16 bit. not set, 8 bit
    HRM_SENSOR_CONTACT = 0x02, // if set, sensor is detected
    HRM_SENSOR_CONTACT_SUPPORT = 0x04, //if set, sensor contact is supported
    HRM_ENERGY_EXPENDED_STATUS = 0x08,
    HRM_RR_INTERVAL_SUPPORT = 0x10,
};

// sensor location
enum blehrm_sensorlocation
{
    HRM_LOC_START = 0x00,
    HRM_LOC_CHEST = 0x01,
    HRM_LOC_WRIST = 0x02,
    HRM_LOC_FINGER = 0x03,
    HRM_LOC_HAND = 0x04,
    HRM_LOC_EARLOBE = 0x05,
    HRM_LOC_FOOT = 0x06,
    HRM_LOC_END = 0x06,
};

// CP Op Code
enum blehrm_opcode
{
    HRM_CP_RESET_ENERGY_EXPENDED = 0x01,
};

typedef PACKED struct
{
    UINT8 flag;
    UINT16 hrm;
    UINT16 ee;
    UINT16 rr[RR_MAX_NUM];
    UINT8 rr_len;
}  BLEHRM_HRM_DATA;

typedef PACKED struct
{
    UINT8     opcode;
} BLEHRM_CP_HDR;

#ifdef _WIN32
#include <poppack.h>
#endif
// GHS syntax.
#pragma pack()

//////////////////////////////////////////////////////////////////////////////
//                      public interface declaration
//////////////////////////////////////////////////////////////////////////////
void blehrm_Create(void);

extern const UINT8 blehrm_db_data[];
extern const UINT16 blehrm_db_size;
extern const BLE_PROFILE_CFG blehrm_cfg;
extern const BLE_PROFILE_PUART_CFG blehrm_puart_cfg;
extern const BLE_PROFILE_GPIO_CFG blehrm_gpio_cfg;

#endif // end of #ifndef _BLEHRM_H_
