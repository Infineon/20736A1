#ifndef _LEATT_H_
#define _LEATT_H_
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
*/

/********************************************************************
 *    File Name: leatt.h
 *
 *    Abstract: This module implements the GATT for LE.
 *
 *    Functions:
 *            --
 *
 *    $History:$
 *
 ********************************************************************
*/

// This is for WIN32 platform.
#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

#define LEATT_ATT_MTU 23


//////////////////////////////////////////////////////////////////////////////
//		constant or data type definition from the ATT/GATT specification.
//
#ifdef _WIN32
#include <pshpack1.h>
#endif
// GHS syntax.
#pragma pack(1)

#define LEATT_HANDLE_RESERVED                                0x0000



#define LEATT_OPCODE_ERR_RSP                                 0x1
#define LEATT_OPCODE_EXCHANGE_MTU_REQ                        0x2
#define LEATT_OPCODE_EXCHANGE_MTU_RSP                        0x3
#define LEATT_OPCODE_FIND_INFO_REQ                           0x4
#define LEATT_OPCODE_FIND_INFO_RSP                           0x5
#define LEATT_OPCODE_FIND_BY_TYPE_VALUE_REQ                  0x6
#define LEATT_OPCODE_FIND_BY_TYPE_VALUE_RSP                  0x7
#define LEATT_OPCODE_READ_BY_TYPE_REQ                        0x8
#define LEATT_OPCODE_READ_BY_TYPE_RSP                        0x9
#define LEATT_OPCODE_READ_REQ                                0xa
#define LEATT_OPCODE_READ_RSP                                0xb
#define LEATT_OPCODE_READ_BLOB_REQ                           0xc
#define LEATT_OPCODE_READ_BLOB_RSP                           0xd
#define LEATT_OPCODE_READ_MULTI_REQ                          0xe
#define LEATT_OPCODE_READ_MULTI_RSP                          0xf
#define LEATT_OPCODE_READ_BY_GROUP_TYPE_REQ                  0x10
#define LEATT_OPCODE_READ_BY_GROUP_TYPE_RSP                  0x11
#define LEATT_OPCODE_WRITE_REQ                               0x12
#define LEATT_OPCODE_WRITE_RSP                               0x13

// 0x14 and 0x15 are not defined.
//

#define LEATT_OPCODE_PREPARE_WRITE_REQ                       0x16
#define LEATT_OPCODE_PREPARE_WRITE_RSP                       0x17
#define LEATT_OPCODE_EXECUTE_WRITE_REQ                       0x18
#define LEATT_OPCODE_EXECUTE_WRITE_RSP                       0x19
// 0x1a is not defined.
#define LEATT_OPCODE_HANDLE_VALUE_NOTIFICATION               0x1b
#define LEATT_OPCODE_HANDLE_VALUE_INDICATION                 0x1d
#define LEATT_OPCODE_HANDLE_VALUE_CONF                       0x1e


// Vol 3, Part F, 3.3.1. Attribute Pdu Format, bit 5-0 Method.
#define LEATT_OPCODE_MASK                                    0x3f
#define LEATT_OPCODE_AUTH_SIGN_FLAG                          0x80
#define LEATT_OPCODE_COMMAND_FLAG                            0x40


typedef PACKED struct
{
    UINT8 attrCode; // attribute code.
} LEATT_PDU_HDR;



#define LEATT_ERR_CODE_NO_ERROR                            0x00

#define LEATT_ERR_CODE_INVALID_HANDLE                      0x01
#define LEATT_ERR_CODE_READ_NOT_PERMITTED                  0x02
#define LEATT_ERR_CODE_WRITE_NOT_PERMITTED                 0x03
#define LEATT_ERR_CODE_INVALID_PDU                         0x04
#define LEATT_ERR_CODE_INSUFFICIENT_AUTHENTICATION         0x05
#define LEATT_ERR_CODE_REQ_NOT_SUPPORTED                   0x06
#define LEATT_ERR_CODE_INVALID_OFFSET                      0x07
#define LEATT_ERR_CODE_INSUFFICIENT_AUTHORIZATION          0x08
#define LEATT_ERR_CODE_PREPARE_QUEUE_FULL                  0x09
#define LEATT_ERR_CODE_ATTRIBUTE_NOT_FOUND                 0x0a
#define LEATT_ERR_CODE_ATTRIBUTE_NOT_LONG                  0x0b
#define LEATT_ERR_CODE_INSUFFICIENT_ENC_KEY_SIZE           0x0c
#define LEATT_ERR_CODE_INVALID_ATTR_VALUE_LENGTH           0x0d
#define LEATT_ERR_CODE_UNLIKELY_ERROR                      0x0e
#define LEATT_ERR_CODE_INSUFFICIENT_ENCRYPTION             0x0f
#define LEATT_ERR_CODE_UNSUPPORTED_GROUP_TYPE              0x10
#define LEATT_ERR_CODE_INSUFFICIENT_RESOURCES              0x11
// 0x12 - 0x7F are reserved.
// 0x80 - 0xff are application error code defined by higher layer specification.
#define LEATT_ERR_CODE_CLIENT_CHAR_CONF_IMPROPERLY         0xfd
#define LEATT_ERR_CODE_PROCEDURE_ALREADY_IN_PROGRESS       0xfe
#define LEATT_ERR_CODE_OUT_OF_RANGE                        0xff

#define LEATT_ERR_CODE_RESERVED_SKIP_AFTER_HOOK            (0x1ul <<31)


#define LEATT_CLIENT_CONFIG_NOTIFICATION                        0x01
#define LEATT_CLIENT_CONFIG_INDICATION                          0x02




typedef PACKED struct
{
    UINT8   attrCode;   // 0x1 = error response
    UINT8   reqOpcode;  // opcode that generate this error.
    UINT16  attrHandleInError;
    UINT8   errCode;    // error code.

} LEATT_PDU_ERR_RSP;

typedef PACKED struct
{
    UINT8   attrCode;   // 0x2 = mtu exchange request
    UINT16  mtu;
} LEATT_PDU_MTU_EXCHANGE_REQ;


typedef PACKED struct
{
    UINT8   attrCode;   // 0x3 = mtu exchange response
    UINT16  mtu;
} LEATT_PDU_MTU_EXCHANGE_RSP;


typedef PACKED struct
{
    UINT8   attrCode;   // 0x4 = find information request
    UINT16  startHandle;
    UINT16  endHandle;
} LEATT_PDU_FIND_INFO_REQ;


#define LEATT_INFO_RSP_FORMAT_UUID16        0x1
#define LEATT_INFO_RSP_FORMAT_UUID128       0x2

typedef PACKED struct
{
    UINT8   attrCode;   // 0x5 = find information response
    UINT8   format;
                        // the data follows the header.
} LEATT_PDU_FIND_INFO_RSP_HDR;

typedef PACKED struct
{
    UINT8   attrCode;   // 0x6 = find by type value request
    UINT16  startHandle;
    UINT16  endHandle;
    UINT16  attrType;
                    // attribute value follows the hdr
} LEATT_PDU_FIND_BY_TYPE_VALUE_REQ_HDR;

typedef PACKED struct
{
    UINT8   attrCode;   // 0x7 = find by type value response
                    // a list of 1 or more handle informations.
} LEATT_PDU_FIND_BY_TYPE_VALUE_RSP_HDR;


typedef PACKED struct
{
    UINT8   attrCode;   // 0x8 = read by type request
    UINT16  startHandle;
    UINT16  endHandle;
        // 2 or 16 byte UUID.
} LEATT_PDU_READ_BY_TYPE_REQ_HDR;

typedef PACKED struct
{
    UINT8   attrCode;   // 0x9 = read by type response
    UINT8   length;
        // attribute data list.
} LEATT_PDU_READ_BY_TYPE_RSP_HDR;


typedef PACKED struct
{
    UINT8   attrCode;   // 0xa = read request
    UINT16  attrHandle;
} LEATT_PDU_READ_REQ_HDR;


typedef PACKED struct
{
    UINT8   attrCode;   // 0xb = read response
        // attribute value
} LEATT_PDU_READ_RSP_HDR;


typedef PACKED struct
{
    UINT8   attrCode;   // 0xc = read blob request
    UINT16  attrHandle;
    UINT16  valueOffset; // offset of the first byte.
} LEATT_PDU_READ_BLOB_REQ;


typedef PACKED struct
{
    UINT8   attrCode;   // 0xd = read blob response
        // part attribute value
} LEATT_PDU_READ_BLOB_RSP_HDR;

typedef PACKED struct
{
    UINT8   attrCode;   // 0xe = read multi request
    // set of 2 or more handles.
} LEATT_PDU_READ_MULTI_REQ_HDR;


typedef PACKED struct
{
    UINT8   attrCode;   // 0xf = read multi response
    // set of 2 or more values.
} LEATT_PDU_READ_MULTI_RSP_HDR;


typedef PACKED struct
{
    UINT8   attrCode;   // 0x10 = read by group type request
    UINT16  startHandle;
    UINT16  endHandle;
        // 2 or 16 bytes of UUID, attribute group type
} LEATT_PDU_READ_BY_GROUP_TYPE_REQ_HDR;


typedef PACKED struct
{
    UINT8   attrCode;   // 0x11 = read by group type response.
    UINT8   length;     // size of earch attribute data
        // a list of attribute data.
} LEATT_PDU_READ_BY_GROUP_TYPE_RSP_HDR;


typedef PACKED struct
{
    UINT8   attrCode;   // 0x12 = Write Req.
    UINT16  attrHandle; // handle to write to.
        // Attribute Value
} LEATT_PDU_WRITE_HDR;


typedef PACKED struct
{
    UINT8   attrCode;   // 0x13 = Write Rsp.
} LEATT_PDU_WRITE_RSP_HDR;


typedef PACKED struct
{
    UINT8   attrCode;     // 0x16 = Prepare write.
    UINT16  attrHandle;   //
    UINT16  valOffset;    //
    // here follows the value.
} LEATT_PDU_PREPARE_WRITE_REQ_HDR;


typedef PACKED struct
{
    UINT8   attrCode;     // 0x17 = prepare write rsp
    UINT16  attrHandle;   //
    UINT16  valOffset;    //
    // here follows the value.
} LEATT_PDU_PREPARE_WRITE_RSP_HDR;



#define LEATT_PDU_EXECUTE_WRITE_REQ_CANCEL       0x0
#define LEATT_PDU_EXECUTE_WRITE_REQ_IMMEDIATE    0x1
typedef PACKED struct
{
    UINT8   attrCode;     // 0x18 = execute write.
    UINT8   flag;         //
} LEATT_PDU_EXECUTE_WRITE_REQ_HDR;


typedef PACKED struct
{
    UINT8   attrCode;     // 0x19 = execute write rsp.
} LEATT_PDU_EXECUTE_WRITE_RSP_HDR;


typedef PACKED struct
{
    UINT8   attrCode;     // 0x1b = Handle Value Notification
    UINT16  handle;
    // here starts the value.
}LEATT_PDU_NOTIFICATION_HDR;



typedef PACKED struct
{
    UINT8   attrCode;     // 0x1b = Handle Value Notification
    UINT16  handle;
    // here starts the value.
}LEATT_PDU_INDICATION_HDR;




typedef PACKED struct
{
  UINT8 len;
  UINT8 pdu[LEATT_ATT_MTU];
} LEATT_INTERNAL_PDU;


typedef PACKED struct
{
    void *msgPtr;// this is the msg ptr.
    INT32   msgLen; // this is the length of the msg.
    void *cb ; // this is the notification callback.
} LEATT_INDICATION_MSG;

typedef void (*LEATT_DOUBLE_PARAM_CB)(INT32 , UINT8 *);
typedef void (*LEATT_TRIPLE_PARAM_CB)(INT32, INT32, UINT8 *);
typedef void (*LEATT_QUADRUPLE_PARAM_CB)(INT32, INT32, INT32, UINT8 *);
typedef void (*LEATT_NO_PARAM_CB)(void);

// Vol 3, Part F, 3.4.5.4
#define LEATT_AUTH_SIGNATURE_LENGTH                          12

// Vol 3. Part F, 2.4.5
#define LEATT_AUTH_SIGNATURE_COUNTER_SIZE                    4


#ifdef _WIN32
#include <poppack.h>
#endif
// GHS syntax.
#pragma pack()


//////////////////////////////////////////////////////////////////////////////
//					Start of function declaration.
//


// called at start up.
void leatt_init(void);

// This will send an ATT Error response.
void leatt_sendErrResponse(INT32 errCode, INT32 reqOpcode, INT32 handleInError );

// This will send an ATT Write Response.
void leatt_sendWriteRsp(void);

// This will send a notification msg.
void leatt_sendNotification(LEATT_PDU_NOTIFICATION_HDR *hdr,INT32 attrLen);

// This will send a indication msg.
void leatt_sendIndication(LEATT_PDU_INDICATION_HDR *hdr,INT32 attrLen,LEATT_NO_PARAM_CB cb);

// This will send a notification msg.
void leatt_sendHandleValueConf(void);


#if ADDL_LEATT_API_SUPPORTED
// This will register callback
void leatt_regFindmeRspCb( LEATT_TRIPLE_PARAM_CB cb);
//Client Functions
//
// This will register callback
void leatt_regHandleValueConfCb(LEATT_NO_PARAM_CB cb);
#endif // ADDL_LEATT_API_SUPPORTED

// This will register callback
void leatt_regReadRspCb( LEATT_TRIPLE_PARAM_CB cb);

// This will register callback
void leatt_regReadByTypeRspCb( LEATT_TRIPLE_PARAM_CB cb);

// This will register callback
void leatt_regReadByGroupTypeRspCb( LEATT_TRIPLE_PARAM_CB cb);

// This will register callback
void leatt_regWriteRspCb(LEATT_NO_PARAM_CB cb);

// This will register callback
void leatt_regNotificationCb( LEATT_TRIPLE_PARAM_CB cb);

// This will register callback
void leatt_regIndicationCb( LEATT_TRIPLE_PARAM_CB cb);

// This will register transaction timeout callback.
void leatt_regTransactionTimeoutCb(LEATT_NO_PARAM_CB  cb);


// This will send a write request
void leatt_sendWriteReq(LEATT_PDU_WRITE_HDR *msg, INT32 len);

// This will send a write command
void leatt_sendWriteCmd(LEATT_PDU_WRITE_HDR *msg, INT32 len);

// This will send a read request
void leatt_sendReadReq(LEATT_PDU_READ_REQ_HDR *msg);

// This will send a read by type Request
void leatt_sendReadByTypeReq(LEATT_PDU_READ_BY_TYPE_REQ_HDR *msg, INT32 len );

// This will send a read by group type Request
void leatt_sendReadByGroupTypeReq(LEATT_PDU_READ_BY_GROUP_TYPE_REQ_HDR *msg, INT32 len);




// This is a convenient function to map 16 bits UUID to 128 bits.
void leatt_mapUUID16ToUUID128(UINT16 uuid, UINT8 *uuid128Holder);

// This function will only update the 2 bytes of UUID.
void leatt_updateUUID16OfUUID128(UINT16 uuid, UINT8 *uuid128Holder);

void leatt_connDown(void );


#endif // end of #ifndef _LEATT_H_
