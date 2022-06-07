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

#ifndef _EMCONINFO_H_
#define _EMCONINFO_H_

/** @file
 *
* This file provides definitions and function prototypes for accessing
* the embedded controller and connection information.
 *
*/

#include "types.h"
#include "bt_rtos.h"

/*****************************************************************************/
/** @defgroup emconinfo       Connection Information
 *
 *  Bluetooth Low Energy Connection Information Functions
 */
/*****************************************************************************/

#ifndef BD_ADDRESS_SIZE
#define BD_ADDRESS_SIZE 6
#endif


#ifdef __cplusplus
extern "C" {
#endif

// This defines the controller states.
// NOT_STARTED means the controller is not binded with the stack.
// STARTUP     means the controller is in the process of binding.
// STARTED     means the controller is binded.
// ADVERTISING means the controller is activated.
// CONNECTION means the controller is in peripheral connection.
typedef enum
{
    NOT_STARTED,
    STARTUP,
    STARTED,
    START_ADVERTISE,
    ADVERTISING,
    CONNECTION
} EMCONINFO_STATE;


#define EMCONINFO_FLAG_BONDED_DEVICE                  0x1
#define EMCONINFO_FLAG_LINK_ENCRYPTED                 0x2

//
typedef struct
{
    // Device Specific section.
    UINT8           addr[BD_ADDRESS_SIZE];          // this is 6 bytes.
    UINT8           flag;                           //
    UINT8           peerClkAccuracy;                // clock accuracy of peer device.

    // 4 bytes aligned.
    UINT16          connHandle;

#define ROLE_CENTRAL                    0
#define ROLE_PERIPHERAL                 1
    UINT8           role;                           // central or peripheral role.
    UINT8           peerAddressType;

    // 4 bytes aligned
    UINT8           peerAddress[BD_ADDRESS_SIZE];   // this is 6 bytes peer address.
    UINT16          connInterval;                   // connection interval in frames.

    // 4 bytes aligned
    UINT16          connLatency;
    UINT16          supervisionTimeout;

    // make
    UINT32          advDuration;                    // This determines how many second adv will last.
    // 4 bytes aligned
    BOOL8           peerDeviceBonded;

    // device address type
    UINT8           addrType;
    INT8            appTimerId;   //
    UINT8           connIdleTimeout; //

    // 4 bytes aligned
    EMCONINFO_STATE     devState;


#ifndef HCIULP_RANDOM_DATA_SIZE
#define HCIULP_RANDOM_DATA_SIZE 8
#endif

    // SMP related section.
    UINT8          rand[HCIULP_RANDOM_DATA_SIZE * 2 ];

    UINT8 discReason;
} EMCONINFO_DEVINFO;

// all the macros need this variable to be exported.
extern EMCONINFO_DEVINFO emConInfo_devInfo;

// gets a pointer to the EMCONINFO_DEVINFO struct
EMCONINFO_DEVINFO *emconinfo_getPtr(void);

// retrieves the disconnect reason
// return code is one of the BT_ERROR codes, see error.h
int emconinfo_getDiscReason(void);

//////////////////////////////////////////////////////////////////////////////
//                          public interfaces.

/**
* \brief Initialize
* \ingroup emconinfo
*
* \details Performs necessary initialization for emconinfo.  Application will
* typically not need to call this function.  It is done as a part of general
* initialization after controller has been initialized.
*
*/
void emconinfo_init(void);


/**
* \brief Get Application Timer ID
* \ingroup emconinfo
*
* \details Returns appTimerId of the Deviceinfo structure.
*
* \return \c Timer
*
*/
INT32  emconinfo_getAppTimerId(void);

void   emconinfo_setAppTimerId(INT32);

UINT32 emconinfo_getAdvDuration(void);
void   emconinfo_setAdvDuration(UINT32 duration);

UINT8  emconinfo_getIdleConnTimeout(void);
void   emconinfo_setIdleConnTimeout(UINT8 t);

#ifdef EMCONINFO_ADDL_API_SUPPORTED
/**
* \brief Get Local Bluetooth Public Device Address
* \ingroup emconinfo
*
* \details This function returns the pointer to the local public device address.
* Note that address is reversed with least significant byte located in offset 0.
*
*/
UINT8  *emconinfo_getPubAddr(void);
#endif

/**
* \brief Get Local Bluetooth Device Address
* \ingroup emconinfo
*
* \details This function returns the pointer to the local device address.
* Note that address is reversed with least significant byte located in offset 0.
*
*/
UINT8 *emconinfo_getAddr(void);

/**
* \brief Get Local Bluetooth Device Address Type
* \ingroup emconinfo
*
* \details This function returns the local device address type.
*
*/
int    emconinfo_getAddrType(void);

//
EMCONINFO_STATE emconinfo_getState(void);
void            emconinfo_setState(EMCONINFO_STATE );

/**
* \brief Get Connection Handle
* \ingroup emconinfo
*
* \details This function returns the HCI handle of the current connection.
*
*/
INT32  emconinfo_getConnHandle(void);
void   emconinfo_setConnHandle(UINT16 connHandle);

/**
* \brief Get Role
* \ingroup emconinfo
*
* \details This function returns the central/peripheral role in the connection
*
* \return
*        \li 0 (ROLE_CENTRAL)
*        \li 1 (ROLE_PERIPHERAL)
*
*/
INT32  emconninfo_getRole(void);
void   emconninfo_setRole(INT32 role);

void   emconninfo_setPeerClkAccuracy(INT32 clk);
INT32  emconninfo_getPeerClkAccuracy(void);

UINT8 *emconninfo_getPeerAddr(void);
UINT8 *emconninfo_getPeerPubAddr(void);

void   emconninfo_setPeerAddrType(INT32 type);
INT32  emconninfo_getPeerAddrType(void);

INT32  emconninfo_getConnInterval(void);
void   emconninfo_setConnInterval(INT32 interval);

INT32  emconninfo_getPeripheralLatency(void);
void   emconninfo_setPeripheralLatency(INT32 latency);

INT32   emconninfo_getSupervisionTimeout(void);
void    emconninfo_setSupervisionTimeout(INT32 timeout);


INT32   emconninfo_deviceBonded(void);
void    emconninfo_setDeviceBondedFlag(void);
void    emconninfo_clearDeviceBondedFlag(void);


INT32  emconninfo_linkEncrypted(void);
void   emconninfo_clearLinkEncryptedFlag(void);
void   emconninfo_setLinkEncryptedFlag(void);

#define EMCONINFO_GET_RAND()                    (&emConInfo_devInfo.rand[0])




// This is for C++ compiler. WIN32 uses C++
#ifdef __cplusplus
}
#endif

#endif // end of #ifndef _EMCONINFO_H_
