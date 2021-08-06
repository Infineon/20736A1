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
* This file provides definitions and function prototypes for the Bluetooth LE
* Battery profile, service, application
*
* Refer to Bluetooth SIG Battery Service 1.0 specification for details.
*
*/
#ifndef _BLEBAT_H_
#define _BLEBAT_H_

#include "adc.h"
#include "lesmp.h"

//////////////////////////////////////////////////////////////////////////////
//                      public data type definition.
//////////////////////////////////////////////////////////////////////////////
#ifdef _WIN32
#include <pshpack1.h>
#endif
// GHS syntax.
#pragma pack(1)

/// Config for the battery monitoring
typedef PACKED struct
{
    /// ADC input to be used for measurement. Note that this may not be a GPIO.
    UINT8 adcInputConnectedToBattery;

    /// Period in millisecs between battery measurements.
    UINT32 measurementInterval;

    /// Number of measurements averaged for a report, max 16.
    UINT8 numberOfMeasurementsToAverage;

    /// The nominal full battery voltage
    UINT16 fullVoltage;

    /// The voltage at which the batteries are considered drained in milli-volts.
    UINT16 emptyVoltage;

    /// System should shutdown if it detects battery voltage at or below this value
    /// Setting to 0 disables shutdown. In millivolts.
    UINT16 shutdownVoltage;

    /// Sets the range of the reported number of steps. Should not be set to 0.
    /// Set it to 100 to report battery in %, 0-100.
    UINT8 maxLevel;

    /// ID of the battery report.
    UINT8 reportID;

    /// Length of the battery report.
    UINT8 reportLength;

    /// Flag indicating that a report should be sent when a connection is established.
    UINT8 reportOnConnect;
}BLEBAT_BATMON_CFG;


enum
{
    /// Maximum number of measurements to average
    MAX_MEAS_TO_AVERAGE = 16
};

// typedef UINT8 BD_ADDR[BD_ADDR_LEN];         /* Device address */

typedef PACKED struct
{
    UINT16 hdl;     // GATT HANDLE number
    UINT16 cl_hdl;  // GATT HANDLE number
    UINT16 ds_hdl;  // GATT HANDLE number
    UINT16 serv;    // GATT service UUID
    UINT16 cha;     // GATT characteristic UUID
    UINT16 des;     // GATT characteristic descriptor UUID
} BLE_BAT_GATT_CFG;

typedef PACKED struct
{
    UINT8    bdAddr[6];         // BD address of the bonded host
    UINT8    notify_mask;       // bitmask of chars that require notifications
    UINT8    broadcast_mask;    // bitmask of chars that require broadcast
} BLEBAT_HOSTINFO;

enum bat_power_state
{
    BLEBAT_POWERSTATE_PRESENT_UNKNOWN               = 0x00,
    BLEBAT_POWERSTATE_PRESENT_NOTSUPPORTED          = 0x01,
    BLEBAT_POWERSTATE_PRESENT_NOTPRESENT            = 0x02,
    BLEBAT_POWERSTATE_PRESENT_PRESENT               = 0x03,
    BLEBAT_POWERSTATE_DISCHARGING_UNKNOWN           = (0x00 << 2),
    BLEBAT_POWERSTATE_DISCHARGING_NOTSUPPORTED      = (0x01 << 2),
    BLEBAT_POWERSTATE_DISCHARGING_NOTDISCHARGING    = (0x02 << 2),
    BLEBAT_POWERSTATE_DISCHARGING_DISCHARGING       = (0x03 << 2),
    BLEBAT_POWERSTATE_CHARGING_UNKNOWN              = (0x00 << 4),
    BLEBAT_POWERSTATE_CHARGING_NOTSUPPORTED         = (0x01 << 4),
    BLEBAT_POWERSTATE_CHARGING_NOTCHARGING          = (0x02 << 4),
    BLEBAT_POWERSTATE_CHARGING_CHARGING             = (0x03 << 4),
    BLEBAT_POWERSTATE_LEVEL_UNKNOWN                 = (0x00 << 6),
    BLEBAT_POWERSTATE_LEVEL_NOTSUPPORTED            = (0x01 << 6),
    BLEBAT_POWERSTATE_LEVEL_GOODLEVEL               = (0x02 << 6),
    BLEBAT_POWERSTATE_LEVEL_CRITICALLYLOWLEVEL      = (0x03 << 6),
};

enum bat_service_required
{
    BLEBAT_SERVICEREQUIRED_UNKNOWN              = 0x00,
    BLEBAT_SERVICEREQUIRED_NOSERVICEREQUIRED    = 0x01,
    BLEBAT_SERVICEREQUIRED_SERVICEREQUIRED      = 0x02,
};

enum bat_removable
{
    BLEBAT_REMOVABLE_UNKNOWN                    = 0x00,
    BLEBAT_REMOVABLE_NOTREMOVABLE               = 0x01,
    BLEBAT_REMOVABLE_REMOVABLE                  = 0x02,
};

#ifdef _WIN32
#include <poppack.h>
#endif
// GHS syntax.
#pragma pack()


//////////////////////////////////////////////////////////////////////////////
//                      public interface declaration
//////////////////////////////////////////////////////////////////////////////
void blebat_Init(void);
void blebat_connUp(void);
void blebat_connDown(void);
void blebat_smpBondResult(LESMP_PARING_RESULT result);
void blebat_pollMonitor(void);



#endif
