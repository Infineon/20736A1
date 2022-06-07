/********************************************************************
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

********************************************************************
*    File Name: additional_gatt_apis.h
*
*    Abstract:  Prvide additional GATT APIs.
*
*    $History:
*           Created   07/22/14
*
*********************************************************************/

//===================================================================
//      Include
//===================================================================
#include <types.h>

//===================================================================
// Functions : bleprofile_sendReadByTypeReq_v2
//
// The function provides a second version bleprofile level API to send ATT "Read By Type Request".
// The original one can only accept 16-bit UUID, but this one can accept 16-bit UUID or 128-bit UUID.
//===================================================================
void bleprofile_sendReadByTypeReq_v2(UINT16 startHandle, UINT16 endHandle, UINT8 uuid[], UINT8 uuid_len);

//===================================================================
// Functions : bleprofile_sendFindByTypeValueReq
//
// The function provides a bleprofile level API to send ATT "Find By Type Value Request".
//===================================================================
void bleprofile_sendFindByTypeValueReq(UINT16 startHandle, UINT16 endHandle, UINT16 attrType, UINT8 attrValue[], UINT8 attrValueLength);

//===================================================================
// Functions : bleprofile_sendFindInfoReq
//
// The function provides a bleprofile level API to send ATT "Find Information Request".
//===================================================================
void bleprofile_sendFindInfoReq(UINT16 startHandle, UINT16 endHandle);

//===================================================================
// Functions : leatt_regFindByTypeValueRspCb
//
// The function provides an API to register the callback function for ATT "Find By Type Value Response".
//===================================================================
void leatt_regFindByTypeValueRspCb(LEATT_DOUBLE_PARAM_CB cb);

//===================================================================
// Functions : leatt_regFindByTypeValueRspCb
//
// The function provides an API to register the callback function for ATT "Find Information Response".
//===================================================================
void leatt_regFindInfoRspCb(LEATT_TRIPLE_PARAM_CB cb);

//===================================================================
// Functions : leatt_regFindByTypeValueRspCb
//
// The function provides an API to register the callback function for ATT "Error Response".
//===================================================================
void leatt_regErrRspCb(LEATT_QUADRUPLE_PARAM_CB cb);
