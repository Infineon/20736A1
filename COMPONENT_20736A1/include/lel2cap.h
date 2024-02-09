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
 */

#ifndef _LEL2CAP_H_
#define _LEL2CAP_H_
/*
 ********************************************************************
 *    File Name: lel2cap.h
 *
 *    Abstract: This module implements the L2CAP for LE. It is a
 *              subset of L2CAP.
 *
 *    Functions:
 *            --
 *
 *    $History:$
 *
 ********************************************************************
*/

#include "types.h"


#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////////////////////////////
// This file defines the data structure for embedded l2cap.
// It is for LE only at this point.

#ifdef _WIN32
#include <pshpack1.h>
#endif
// GHS syntax.
#pragma pack(1)


#define LEL2CAP_MTU                     23

// Here are the defined Channel ID.
#define LEL2CAP_CID_NULL                0x0
#define LEL2CAP_CID_SIGNALING           0x1
#define LEL2CAP_CID_CONNECTIONLESS      0x2
#define LEL2CAP_CID_AMP                 0x3
#define LEL2CAP_CID_LE_ATT              0x4
#define LEL2CAP_CID_LE_SIGNALING        0x5
#define LEL2CAP_CID_LE_SMP              0x6

#define LEL2CAP_CID_LE_AP                  0x2A //42

// this is the general header for L2CAP.
typedef PACKED struct
{
    UINT16 length;
    UINT16 cid;
} LEL2CAP_HDR;


#define LEL2CAP_GET_LENGTH( a )     ( (a)->length )



#define LEL2CAP_RESERVED                            0x0    //
#define LEL2CAP_REJECTED                            0x1    //

#define LEL2CAP_CONN_REQ                            0x2    //
#define LEL2CAP_CONN_RSP                            0x3    //

#define LEL2CAP_CONN_FIG_REQ                        0x4    //
#define LEL2CAP_CONN_FIG_RSP                        0x5    //

#define LEL2CAP_DISCONN_REQ                         0x6    //
#define LEL2CAP_DISCONN_RSP                         0x7    //

#define LEL2CAP_ECHO_REQ                            0x8    //
#define LEL2CAP_ECHO_RSP                            0x9    //

#define LEL2CAP_INFO_REQ                            0xa    //
#define LEL2CAP_INFO_RSP                            0xb    //

#define LEL2CAP_CREATE_CHANNEL_REQ                  0xc    //
#define LEL2CAP_CREATE_CHANNEL_RSP                  0xd    //

#define LEL2CAP_MOVE_CHANNEL_REQ                    0xe    //
#define LEL2CAP_MOVE_CHANNEL_RSP                    0xf    //

#define LEL2CAP_MOVE_CHANNEL_CONFIRM_REQ            0x10   //
#define LEL2CAP_MOVE_CHANNEL_CONFIRM_RSP            0x11    //

#define LEL2CAP_CONN_PARAM_UPDATE_REQ               0x12   // valid for cid 5
#define LEL2CAP_CONN_PARAM_UPDATE_RSP               0x13   // valid for cid 5




// this is the command packet header.
typedef PACKED struct
{
    UINT16 length;
    UINT16 cid;
    UINT8  code;
    UINT8  id;
    UINT16 len;   // this is the length of the payload data.
} LEL2CAP_COMMAND_HDR;


// this is connection parameter update.
typedef PACKED struct
{
    UINT16 length;
    UINT16 cid;
    UINT8  code;
    UINT8  id;
    UINT16 len;
    UINT16 result;
} LEL2CAP_CMD_RSP_PARAM_UPDATE;


typedef PACKED struct
{
    UINT8 code;
    UINT8 identifier;
    UINT16 length;
    UINT16 reason;
    // optional data.
} LEL2CAP_CMD_REJECT_HDR;

#if 1
typedef PACKED struct
{
    UINT8  code;      // code = 0xa
    UINT8  identifier;
    UINT16 length;
    UINT16 infoType;
} LEL2CAP_CMD_INFO_REQ;
#endif

typedef PACKED struct
{
    UINT8 code;      // code = 0xb
    UINT8 identifier;
    UINT16 length;
    UINT16 infoType;
    UINT16 result;
    // optional data.
} LEL2CAP_CMD_INFO_RSP_HDR;


typedef PACKED struct
{
    UINT8 code;      // code = 0x12
    UINT8 identifier;
    UINT16 length;
    UINT16 minInterval;
    UINT16 maxInterval;
    UINT16 peripheralLatency;
    UINT16 timeout;
} LEL2CAP_CMD_CONN_PARAM_UPDATE_REQ;

typedef PACKED struct
{
    UINT8 code;      // code = 0x13
    UINT8 identifier;
    UINT16 length;
    UINT16 result;
} LEL2CAP_CMD_CONN_PARAM_UPDATE_RSP;

#ifdef _WIN32
#include <poppack.h>
#endif
// GHS syntax.
#pragma pack()

// These are used for command reject.
#define LEL2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD                     0x0000
#define LEL2CAP_CMD_REJECT_REASON_SIG_MTU_EXCEEDED                   0x0001
#define LEL2CAP_CMD_REJECT_REASON_INVALID_CID_IN_REQ                 0x0002


// These are used for Info Request
#define LEL2CAP_INFO_REQ_CONNECTIONLESS_MTU                          0x1
#define LEL2CAP_INFO_REQ_EXTENDED_FEATURES_SUPPORTED                 0x2
#define LEL2CAP_INFO_REQ_FIXED_CHANNELS_SUPPORTED                    0x3


// These are used for Info Response
#define LEL2CAP_INFO_RSP_REASON_SUCCESS                              0x0
#define LEL2CAP_INFO_RSP_REASON_NOT_SUPPORTED                        0x1
typedef PACKED struct
{
    UINT16     connectionHandle;
    UINT16                        length;
} HCI_ACL_HEADER1;
// This is the acl data handler. It is for lower layer.
typedef void (*LEL2CAP_ACLHANDLER)(HCI_ACL_HEADER1 *);

// This is the l2cap data handler. It is for upper layer.
typedef void (*LEL2CAP_L2CAPHANDLER)(LEL2CAP_HDR *);




//////////////////////////////////////////////////////////////////////////////
//                  Start of function declaration.
//


// called at start up.
void lel2cap_init(void);

// this interce register a call back for the connection less channel
// handler.
void lel2cap_regConnLessHandler( int cid, LEL2CAP_L2CAPHANDLER handler);
void lel2cap_regConnLessAudioHandler( int cid, LEL2CAP_L2CAPHANDLER handler);
void lel2cap_regAudioStreamHandler(LEL2CAP_ACLHANDLER handler);

// This function will send a l2cap packet out through connectionless
// Channel Id.
void lel2cap_sendConnectionLessPkt(int cid, UINT8 *pkt, int len );

void lel2cap_sendConnParamUpdateReq(UINT16 minInterval,
            UINT16 maxInterval, UINT16 peripheralLatency, UINT16 timeout);


#ifdef __cplusplus
} // end of the braces for c++.
#endif

#endif // end of #ifndef _LEL2CAP_H_
