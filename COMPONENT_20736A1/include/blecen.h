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

#ifndef _BLECEN_H_
#define _BLECEN_H_

/** @file
*
* This file implements the Bluetooth LE generic central profile, service, application
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

#define ADV_INFO_MAX 5

#define SCANRSP_LEN_MAX 17
#define WD_ADDRESS_SIZE 6

#define HCIULP_MAX_DATA_LENGTH                                          0x1F

#define HCIULP_PASSIVE_SCAN                                             0x00
#define HCIULP_ACTIVE_SCAN                                              0x01

#define HCIULP_PUBLIC_ADDRESS                                           0x00
#define HCIULP_RANDOM_ADDRESS                                           0x01

#define HCIULP_SCAN_MODE_OFF                                            0x00
#define HCIULP_SCAN_MODE_ON                                             0x01

#define HCIULP_SCAN_DUPLICATE_FILTER_OFF                                0x00
#define HCIULP_SCAN_DUPLICATE_FILTER_ON                                 0x01

#define HCIULP_INITIATOR_MODE_OFF                                       0x00
#define HCIULP_INITIATOR_MODE_ON                                        0x01                       // For create connection scan

#define HCIULP_SCAN_FILTER_DUMPLICATES_OFF                              0
#define HCIULP_SCAN_FILTER_DUMPLICATES_ON                               1

#define HCIULP_SCAN_FILTER_POLICY_ACCEPT_LIST_NOT_USED                  0x00    // filter accept list not used
#define HCIULP_SCAN_FILTER_POLICY_ACCEPT_LIST_USED                      0x01    // filter accept list used

#define HCIULP_INITIATOR_FILTER_POLICY_ACCEPT_LIST_NOT_USED             0x00    // filter accept list not used
#define HCIULP_INITIATOR_FILTER_POLICY_ACCEPT_LIST_USED                 0x01    // filter accept list used

enum ble_scan_mode
{
    NO_SCAN = 0x00,
    LOW_SCAN = 0x01,
    HIGH_SCAN = 0x02,
    PASIVE_SCAN = 0x00,
    ACTIVE_SCAN = 0x04,
};

enum ble_conn_mode
{
    NO_CONN = 0x00,
    LOW_CONN = 0x01,
    HIGH_CONN = 0x02,
};

enum ble_conn_speed_mode
{
    FAST_CONN = 0x00,
    SLOW_CONN = 0x01,
};

typedef PACKED struct
{
    UINT8 scan_type;
    UINT8 scan_adr_type;
    UINT8 scan_filter_policy;
    UINT8 filter_duplicates;
    UINT8 init_filter_policy;
    UINT8 init_addr_type;
    UINT16 high_scan_interval; //slots
    UINT16 low_scan_interval; //slots
    UINT16 high_scan_window; //slots
    UINT16 low_scan_window; //slots
    UINT16 high_scan_duration; //seconds
    UINT16 low_scan_duration; //seconds
    UINT16 high_conn_min_interval;  //frames
    UINT16 low_conn_min_interval;  //frames
    UINT16 high_conn_max_interval; //frames
    UINT16 low_conn_max_interval; //frames
    UINT16 high_conn_latency; //number of connection event
    UINT16 low_conn_latency; //number of connection event
    UINT16 high_supervision_timeout; // N * 10ms
    UINT16 low_supervision_timeout; // N * 10ms
    UINT16 conn_min_event_len; // slots
    UINT16 conn_max_event_len; // slots
} BLE_CEN_CFG;

typedef PACKED struct
{
    BD_ADDR peer_addr;
    UINT8 peer_addr_type;
    UINT8 flags;
    UINT16 uuid16;
    UINT8 tx_level;
    UINT8 name[ADV_LEN_MAX];
}BLE_CEN_ADV_INFO;

typedef unsigned char    HCI_BYTE;
typedef signed char      HCI_SBYTE;
typedef unsigned short   HCI_WORD;
typedef signed short     HCI_SWORD;
typedef unsigned long    HCI_DWORD;

typedef PACKED struct
{
    HCI_BYTE             eventType;
    HCI_BYTE             addressType;
    HCI_BYTE             wd_addr[WD_ADDRESS_SIZE];
    HCI_BYTE             dataLen;
    HCI_BYTE             data[64];
    HCI_SBYTE            rssi;

} HCIULP_ADV_PACKET_REPORT_WDATA;

#ifdef _WIN32
#include <poppack.h>
#endif
// GHS syntax.
#pragma pack()

//////////////////////////////////////////////////////////////////////////////
//                      public interface declaration
//////////////////////////////////////////////////////////////////////////////
void blecen_Create(void);
#if BLECEN_ADDL_API_SUPPORTED
void blecen_Init(LEATT_TRIPLE_PARAM_CB cb);
#endif
UINT32 blecen_CenButton(UINT32 function);

UINT8 blecen_GetScan(void);
void blecen_Scan(UINT8 scan_mode);
void blecen_ScanTimerCb( UINT32 arg);

UINT8 blecen_GetConn(void);
void blecen_Conn(UINT8 conn_mode, BD_ADDR peer_addr, UINT8 peer_addr_type);
void blecen_ConnTimerCb(UINT32 arg);
void blecen_appTimerCb(UINT32 arg);

void blecen_ConnUpdate(UINT8 conn_speed);
void blecen_connDown(void);
void blecen_smpBondResult(LESMP_PARING_RESULT result);
void blecen_encryptionChanged(HCI_EVT_HDR *evt);
void blecen_leAdvReportCb(HCIULP_ADV_PACKET_REPORT_WDATA *evt);

extern const BLE_PROFILE_CFG blecen_cfg;
extern const BLE_PROFILE_PUART_CFG blecen_puart_cfg;
extern const BLE_PROFILE_GPIO_CFG blecen_gpio_cfg;

#endif // end of #ifndef _BLECEN_H_
