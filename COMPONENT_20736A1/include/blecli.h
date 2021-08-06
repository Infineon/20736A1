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

#ifndef _BLECLI_H_
#define _BLECLI_H_
/*******************************************************************************
*
* File Name: blecli.h
*
* Abstract: This file implements the Bluetooth LE Generic client profile, service, application
*
*
* Functions:
*
*******************************************************************************/
#include "bleprofile.h"



//////////////////////////////////////////////////////////////////////////////
//                      public data type definition.
//////////////////////////////////////////////////////////////////////////////
#ifdef _WIN32
#include <pshpack1.h>
#endif
// GHS syntax.
#pragma pack(1)

#define CLIENT_HANDLE_NUM_MAX 3

// Client handle
typedef PACKED struct
{
    UINT16 svc_uuid;
    UINT16 cha_uuid;
    UINT16 desc_uuid;
    UINT16 cha_hdl;
    UINT16 desc_hdl;
}  BLE_CLIENT_HANDLE;



#ifdef _WIN32
#include <poppack.h>
#endif
// GHS syntax.
#pragma pack()


//////////////////////////////////////////////////////////////////////////////
//                      public interface declaration
//////////////////////////////////////////////////////////////////////////////
void blecli_Init(void);
UINT8 blecli_ClientButton(UINT16 svc_uuid, UINT16 cha_uuid, UINT16 desc_uuid,
	UINT8 action, UINT8 *data, UINT8 len, LEATT_TRIPLE_PARAM_CB cb);
void blecli_ClientHandleReset(void);
void blecli_ClientReq(void);
#ifdef BLECLI_ADDL_API_SUPPORTED
void blecli_ClientHandleTest(void);
#endif
INT8 blecli_ClientGetIndex(UINT16 svc_uuid, UINT16 cha_uuid, UINT16 desc_uuid);
UINT16 blecli_ClientGetHandle(UINT16 svc_uuid, UINT16 cha_uuid, UINT16 desc_uuid);
UINT8 blecli_ClientSetHandle(UINT16 svc_uuid, UINT16 cha_uuid, UINT16 desc_uuid, UINT16 hdl);
UINT8 blecli_ClientReplaceHandle(INT8 index, UINT16 desc_uuid, UINT16 new_hdl);

#endif // end of #ifndef _BLECLI_H_
