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

#ifndef _LESMP_H_
#define _LESMP_H_
/*
******************************************************************************
*    File Name: lesmp.h
*
*    Abstract: This is header file for LE SMP. It is a simplified version
*               for LE.
*
*    $History:$
*
******************************************************************************
*/
#include "lesmp_const.h"
#include "lesmpkeys.h"
#include "blecm.h"
#include "lel2cap.h"

#ifdef __cplusplus
extern "C" {
#endif



//////////////////////////////////////////////////////////////////////////////
//
typedef enum
{
    LESMP_IDLE,
    LESMP_W4_M_CONF,
    LESMP_W4_M_RAND,
    LESMP_W4_L_RAND,
    LESMP_W4_LOCAL_CONF_C1_1, // start to generate local Confirmation.
    LESMP_W4_LOCAL_CONF_C1_2, // wait for PassKey.
    LESMP_W4_LOCAL_CONF_C1_3, // encrypt p1.
    LESMP_W4_LOCAL_CONF_C1_4, // encrypt p2.
    LESMP_W4_VERIFY_M_CONF_1,
    LESMP_W4_VERIFY_M_CONF_2,
    LESMP_W4_KEY,             // wait for the key.
    LESMP_W4_LINK_ENC,
    LESMP_W4_LINK_CHANGED,
    LESMP_LINK_ENCRYPTED,
    LESMP_SMP_TIMEOUT          // This state prevents pairing from happening.
} LESMP_STATE;

typedef enum
{
    LESMP_W4_PAIRING_RSP=100,
    LESMP_W4_S_CONF=101,
    LESMP_W4_S_RAND=102,
    LESMP_W4_VERIFY_S_CONF_1=103,
    LESMP_W4_VERIFY_S_CONF_2=104,
    LESMP_W4_E_RAND=105,
} LESMP_INITIATOR_STATE;

#define LESMP_PAIRING_MODE_RESERVED                         0x0
#define LESMP_PAIRING_MODE_JUST_WORKS                       0x1
#define LESMP_PAIRING_MODE_PASS_KEY_RESPONDER_DISPLAY       0x2
#define LESMP_PAIRING_MODE_PASS_KEY_RESPONDER_INPUT         0x3
#define LESMP_PAIRING_MODE_PASS_KEY_BOTH_INPUT              0x4
#define LESMP_PAIRING_MODE_OOB_PAIRING                      0x5


#ifdef _WIN32
#include <pshpack1.h>
#endif
// GHS syntax.
#pragma pack(1)

typedef PACKED struct
{
    //
    UINT8 smpRole;
    UINT8 usedKeySize;
    UINT8 pairingMode;
    UINT8 bondingIdx;

    // This has the default pairing parameter.
    LESMP_PAIRING_RSP_CMD pairingParam;

    LESMP_PAIRING_REQ_CMD pairingReqCmd;
    LESMP_PAIRING_RSP_CMD pairingRspCmd;

    // rand. 16 bytes.
    UINT8 rand[LESMP_RAND_SIZE];
    UINT8 randBytes;

    // this is the hci operation that we are waiting for.
    UINT16 pendingHCIOpCode;
    BOOL8  pendingHCIOperation;

    // confirmation. 16 bytes.
    UINT8 peerConfirmation[LESMP_RAND_SIZE];
    UINT8 peerRand[LESMP_RAND_SIZE];

    // STK
    UINT8 stk[LESMP_MAX_KEY_SIZE];

    // TK
    UINT8 tk[LESMP_MAX_KEY_SIZE];

    BOOL8 linkEncrypted;
    BOOL8 startBonding;
    INT8  smpTimerId;
    UINT8 smpTimeOut; // it should be 30 seconds.Vol
    UINT8 exchangedKeys;
    UINT8 numOfBondedDevices; //
    LESMP_STATE state;
    UINT8       lesmp_passKey[16];
    LESMPKEYS_BONDED_INFO lesmpkeys_bondedInfo;
} LESMP_INFO;



typedef enum
{
   LESMP_PAIRING_RESULT_FAILED,
   LESMP_PAIRING_RESULT_TIMEOUT,
   LESMP_PAIRING_RESULT_SUCCESS,   // this does not include bonding.
   LESMP_PAIRING_RESULT_BONDED    // this implies pairing succes.
} LESMP_PARING_RESULT;

typedef void (*LESMP_SINGLE_PARAM_CB)(int );
typedef void (*LESMP_RAND_CB)(char *);


#ifdef _WIN32
#include <poppack.h>
#endif
// GHS syntax.
#pragma pack()

//////////////////////////////////////////////////////////////////////////////
// global variables.
//extern LESMP_STATE lesmp_state;
extern LESMP_INFO  lesmp_info;
extern LESMP_INFO  *lesmp_pinfo;
extern UINT8 lesmp_minKeySize;


// initialization.
void lesmp_init(LESMP_INFO  *pinfo);

void lesmp_setPtr(LESMP_INFO  *pinfo);
LESMP_INFO  * lesmp_getPtr(void);

#define LESMP_ROLE_RESPONDERS              0x0
#define LESMP_ROLE_INITIATOR               0x1

//
void lesmp_setSMPRole(int role);
void lesmp_setPairingParam(
    int   IOCapability,
    int   OOBDataFlag,
    int   AuthReq,
    int   MaxEncKeySize,
    int   InitiatorKeyDistrib,
    int   ResponderKeyDistrib);

void lesmp_setPairingMode(LESMP_PAIRING_REQ *pkt );

void lesmp_setPassKey(UINT8 *passKey, int len);

void lesmp_setSMPOOBdata(UINT8 *oob, int len );

void lesmp_setSMPassKey(UINT8 *passkey, int len );

//
void lesmp_getRand(void);

void lesmp_getRandwithCb(LESMP_RAND_CB cb);

// le_encrypt
void lesmp_leEncrypt(UINT8 *key, UINT8 *text);

void lesmp_calcLocalConf(UINT8 *ptr);

void lesmp_calcSConf(UINT8 *key, UINT8 *rand);

// void lesmp_procCmdCompleteEvt(HCI_EVENT_COMMAND_COMPLETE *evt );

void lesmp_verifyMConf(UINT8 *key, UINT8 *rand);

void lesmp_xor(UINT8 *dst, UINT8 *src, int len);

void lesmp_keygenS1(UINT8 *key);
void lesmp_getLTK(void);

void lesmp_sendLTKReply(UINT8 *key, UINT8 *rand, UINT8 *ediv);
void lesmp_sendLTKNegReply(void);

void lesmp_formP1(UINT8 *p);
void lesmp_formP2(UINT8 *p);

void lesmp_sendLocalRand(void);
void lesmp_sendConfirmation(UINT8 *pkt);

void lesmp_calcLocalMConf(UINT8 *ptr);
void lesmp_calcMConf(UINT8 *key, UINT8 *rand);
void lesmp_verifySConf(UINT8 *key, UINT8 *rand);

int lesmp_startPairing(UINT8 *AuthReq);
void lesmp_sendPairingReq(void);

void lesmp_sendPairingRsp(void);
void lesmp_sendConfirmation(UINT8 *pkt);
void lesmp_sendLocalRand(void);
void lesmp_sendPairingFailed(int);

void lesmp_sendEncInfo( UINT8 *ltk );
void lesmp_sendCentralId( UINT8 *rand, UINT16 ediv );
void lesmp_sendIdInfo(UINT8 *irk );
void lesmp_sendIdAddrInfo(UINT8 *bdAddr, int type );
void lesmp_sendSigningInfo(UINT8 *csrk );


void lesmp_sendSecurityRequest(void);

// This function uses software AES engine.
void lesmp_aesEncrypt(UINT8 *key, UINT8 *text, UINT8* outText);

void lesmp_log(char *str );
void lesmp_log1(char *str, int val );
void lesmp_logBytes(char *str, UINT8 *data, int len);

INT8 lesmp_getTimerId(void);
void lesmp_setTimerId(INT8 id);

void  lesmp_setTimeout(UINT8 timeout);
UINT8 lesmp_getTimeout(void);

int  lesmp_linkEncrypted(void);

// These 3 functions take care of the SMP timer.
void lesmp_startSMPTimer(void);
void lesmp_stopSMPTimer(void);
void lesmp_refreshSMPTimer(void);

void lesmp_connUp(void);
void lesmp_connDown(void);

void lesmp_regSMPResultCb( LESMP_SINGLE_PARAM_CB cb);


void lesmp_recvKey(UINT8 keyMask);

int  lesmp_verifySignature( UINT8 *key, UINT8 *msg, int msgLen,
        UINT8 *mac, int mscLen );

// This function will do the AES-CMAC calculation.
void lesmp_cmac(UINT8 *key, UINT8 *msg, int len, UINT8*mac, int macLen );

void lesmp_endianFlipInPlace(UINT8 *buf);

void lesmp_endianFlipCopy(UINT8 *in, UINT8 *out, int len );


int  lesmp_checkEncryptionKeySize(void);
void lesmp_setCheckEncryptionKeySize(void);

void lesmp_l2capHandler(LEL2CAP_HDR *l2capHdr);

#define LESMP_INVALID_BONDING_INDEX    0xff

#ifdef __cplusplus
}
#endif
#endif // end of #ifndef _LESMP_H_
