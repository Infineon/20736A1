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

#ifndef __BLE_APP_CONFIG__
#define __BLE_APP_CONFIG__

#include "types.h"
#include "bleprofile.h"
#include "hidddriversconfig.h"

#pragma pack(1)
/// Miscellaneous configuration items for applications.
/// Currently, only the application dedicated area in the EEPROM is defined here
/// Items that do not logically fit anywhere maybe added to this.
typedef PACKED struct
{
    /// Port and Pin to which EEPROM WP is connected
    /// Most significant 3 bits - port;
    /// Least significant 5 bits - pin;
    /// 0xFF when WP is not configured
    BYTE bleapputils_eepromWpPortPin;

    /// Location of the application dedicated area in the EEPROM
    UINT16 bleapputils_eepromCustAreaOffset;

    /// Size of the application dedicated area in the EEPROM
    UINT16 bleapputils_eepromCustAreaLen;
} MiscBleAppConfig;

#pragma pack()

extern BLE_PROFILE_CFG bleprofile_cfg;
extern BLE_PROFILE_PUART_CFG bleprofile_puart_cfg;
extern BLE_PROFILE_GPIO_CFG bleprofile_gpio_cfg;
extern BLE_PROFILE_PWM_CFG bleprofile_pwm_cfg;
extern BLEAPP_INIT_CFG bleapp_init_cfg;
extern BLEAPP_DB_CFG bleapp_db_cfg;
extern UINT8* bleapp_p_db;
extern BLEAPP_SELECT_LPO_CFG bleapp_select_lpo_cfg;
extern BLEAPP_CPU_CLOCK_CFG bleapp_cpu_clock_cfg;
extern BLEBAT_BATMON_CFG blebat_batmon_cfg;
extern UINT8 bleapp_trace_enable;
extern UINT8 bleapp_hidoff_enable;
extern UINT32 bleapp_max_sleep;
#endif
