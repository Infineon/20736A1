#ifndef _LESMPKEYS_H_
#define _LESMPKEYS_H_

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
*
******************************************************************************
*    File Name: lesmpkeys.h
*
*    Abstract: This is the header file for LE SMP keys. We use ER/IR for
*              key management.
*
*
*    $History:$
*
******************************************************************************
*/

//
#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
//#include "lesmp.h"
#include "lesmp_const.h"

#define LESMPKEYS_MAX_BONDED_DEVICES        5

// This address size should always be 6 as defined in Bluetooth Spec.
#define LESMP_BD_ADDR_SIZE                  6




//////////////////////////////////////////////////////////////////////////////
//                  Security Key data structure.

#ifdef _WIN32
#include <pshpack1.h>
#endif
// GHS syntax.
#pragma pack(1)

// Long Term Key.
typedef PACKED struct
{
    UINT8  key[LESMP_MAX_KEY_SIZE];    // this is 16 bytes.
    UINT8  rand[LESMP_MAX_KEY_SIZE/2]; // this is 8 bytes.
    UINT16 ediv;
    UINT8  ltkSize; // this is the size of LTK.
    UINT8 _padding; // this is to pad the structure to be 4 bytes.
} LESMP_KEY_LTK;

// This is the key array.
typedef PACKED struct
{
    UINT8 data[LESMP_MAX_KEY_SIZE];
} LESMPKEYS_KEY;

#if 0
// signing key.
typedef PACKED struct
{
    UINT8 key[LESMP_MAX_KEY_SIZE];
} LESMP_KEY_CSRK;
#endif

// This bit means the pairing was done with Authentication, i.e Not JustsWorks.
#define LESMPKEYS_AUTH_PAIRED            0X80
#define LESMPKEYS_PAIRED_KEYSIZE         0X1f

// This means the entry is valid.
#define LESMP_SECURITY_KEY_FLAGS_VALID                        0x0001

typedef PACKED struct
{
    UINT8           address[LESMP_BD_ADDR_SIZE]; // this is 6 bytes.
    UINT16          ediv;     // this is peer ediv.
    // 4 byte aligned.
    UINT8           adrType;  // address type.
    UINT8           authInfo; // MITM at [7] + key size.
    UINT8           _reserv2; // address type.
    UINT8           _reserv3; // address type.
    // 4 byte aligned.
    UINT8           randPeer[LESMP_MAX_KEY_SIZE/2];  // this is peer rand.
    // 4 byte aligned.
    LESMPKEYS_KEY   irkPeer;  // this is peer irk.
    LESMPKEYS_KEY   csrkPeer; // this is peer csrk.
    UINT32          peerCounter; // this is the sign counter.
    LESMPKEYS_KEY   ltkPeer;  // this is peer ltk.

    LESMPKEYS_KEY   irk;  // this is local irk.
    LESMPKEYS_KEY   csrk; // this is local csrk.
    LESMPKEYS_KEY   ltk;  // this is local ltk.

} LESMPKEYS_BONDED_INFO;  //LESMP_SECURITY_KEYS;


typedef enum
{
   LESMPKEYS_STORAGE_TYPE_LTK       = 0x0,
   LESMPKEYS_STORAGE_TYPE_LTK_EDIV  = 0x1,
   LESMPKEYS_STORAGE_TYPE_LKT_RAND  = 0x2,
   LESMPKEYS_STORAGE_TYPE_IRK       = 0x3,
   LESMPKEYS_STORAGE_TYPE_CSRK      = 0x4,

} LESMPKEYS_STORAGE_TYPE;


#define LESMPKEYS_LOCAL_ROOT_KEY_VALID_IR              0x1
#define LESMPKEYS_LOCAL_ROOT_KEY_VALID_ER              0x2
#define LESMPKEYS_LOCAL_ROOT_KEY_VALID_IRK              0x4
typedef struct
{
    UINT32 flag;
    UINT32 bondedIdxMask;         // this is the idx mask.
    UINT8 IR[LESMP_MAX_KEY_SIZE]; // Identity Root.
    UINT8 ER[LESMP_MAX_KEY_SIZE]; // Encryption Root.
    UINT8 IRK[LESMP_MAX_KEY_SIZE]; // IRK
} LESMPKEYS_LOCAL_ROOT_KEY;


typedef PACKED struct
{
    UINT8 address[LESMP_BD_ADDR_SIZE]; // this is 6 bytes.
    UINT8 adrType;
    UINT8 idx; // this is the idx of the actual info.
} LESMPKEYS_BOND_IDX;


#ifdef _WIN32
#include <poppack.h>
#endif
// GHS syntax.
#pragma pack()


// This function should be called at boot up.
void lesmpkeys_init(void);

// these function returns true is IR is valid.
BOOL8 lesmpkeys_validIR(void);

// these function returns true is ER is valid.
BOOL8 lesmpkeys_validER(void);

// these function returns true is IRK is valid.
BOOL8 lesmpkeys_validIRK(void);

// This function should called after NVRAM info has been loaded.
void lesmpkeys_setupLocalKeys(void);

// This is an function defined in 5.2.2.1 Diversifying function d1.
void lesmpkeys_d1(UINT8 *key, UINT16 div, UINT16 r, UINT8 *outData);


// This function will return the index of the key.
// -1 if it is not in the storage.
int lesmpkeys_find(UINT8 *bdAddr, int adrType);

// This function will allocate a storage space to store keys.
int lesmpkeys_allocKeyStorageIdx(void);

// This function will remove the bonded info for the addess.
void lesmpkeys_deleteBondInfo( UINT8 * bdAddr, int adrType);

// This function set the keys, storage type indicates what kind of storage
// to store.
int lesmpkeys_setKey(int idx,LESMPKEYS_STORAGE_TYPE storageType, UINT8 *inData);

// LESMPKEYS_STORAGE_TYPE indicates what type of storage pointer to get.
UINT8 *lesmpkeys_getKeyPtr(int idx,LESMPKEYS_STORAGE_TYPE storageType);

// This will store the RAM info to flash.
int lesmpkeys_storeBondedInfo(int idx);

// This function will load the device info from persistant storage.
// If the load did not happen either due to no such device or persistant
// storage is not available, it returns FALSE. Otherwise it returns TRUE.
int lesmpkeys_loadDevInfo(UINT8 *bdAddr, int adrType);


void lesmpkeys_deriveKeysFromRootKey(void);


// This function will remove All Bonded info.
void lesmpkeys_removeAllBondInfo( void );
int  lesmpkeys_numOfBondedDevice(void);
int  lesmpkeys_maxNumOfBondedDevice(void);


//////////////////////////////////////////////////////////////////////////////
// We may need to modify this storage module frequently.
typedef int (*LESMPKEYS_API)(void * );
typedef int (*LESMPKEYS_API_2)(void *, int );

typedef struct
{
    LESMPKEYS_API init; // This will init the storage module.
    LESMPKEYS_API find; // This will look up the bd address.
    LESMPKEYS_API alloc; // This will allocate a storage space.
    LESMPKEYS_API set;  // This will store the info by idx.
    LESMPKEYS_API get;  // This will be to get the pointer by idx.
    LESMPKEYS_API storeBondedInfo; // This stores bonded info.
} LESMPKEYS_API_TABLE;





// This will be used to access the handler.
extern LESMPKEYS_API_TABLE *lesmpkeys_apiTablePtr;

//extern LESMPKEYS_BONDED_INFO lesmpkeys_bondedInfo;

extern LESMPKEYS_LOCAL_ROOT_KEY lesmpkeys_localRootKey;

//////////////////////////////////////////////////////////////////////////////
//     public interface.
#define LESMPKEYS_API_INIT() (lesmpkeys_apiTablePtr->init() )

#define LESMPKEYS_API_FIND(a,b) \
    (( (LESMPKEYS_API_2)(lesmpkeys_apiTablePtr->find))((a),(b)) )

#define LESMPKEYS_API_ALLOC(a) (lesmpkeys_apiTablePtr->alloc((a)) )

#define LESMPKEYS_API_SET_KEY_INFO(a,b,c) (lesmpkeys_apiTablePtr->set((a),(b),(c)) )

#define LESMPKEYS_API_GET_KEY_INFO(a,b) (lesmpkeys_apiTablePtr->set((a),(b)) )

#define LESMPKEYS_API_SERIALIZE(a) (lesmpkeys_apiTablePtr->storeBondedInfo((void *)(a)) )



//////////////////////////////////////////////////////////////////////////////
//          This function will set the local LTK.
#if ADDL_LESMPKEYS_API_SUPPORTED
void    lesmpkeys_setLocalLTK(UINT8 *ltk);
#endif
UINT8*  lesmpkeys_getLocalLTK(void);

void    lesmpkeys_setPeerLTK(UINT8 *ltk);
UINT8*  lesmpkeys_getPeerLTK(void);

#if ADDL_LESMPKEYS_API_SUPPORTED
void    lesmpkeys_setLocalEDIV(UINT8 *rand, UINT16 ediv);
#endif
UINT16  lesmpkeys_getLocalEDIV(void);
UINT8*  lesmpkeys_getLocalRand(void); // this is the rand that goes with ediv.

#if ADDL_LESMPKEYS_API_SUPPORTED
void    lesmpkeys_setLocalIRK(UINT8 *irk);
#endif
UINT8*  lesmpkeys_getLocalIRK(void);

void    lesmpkeys_setPeerIRK(UINT8 *irk);
UINT8*  lesmpkeys_getPeerIRK(void);

#if ADDL_LESMPKEYS_API_SUPPORTED
void    lesmpkeys_setLocalCSRK(UINT8 *csrk);
#endif
UINT8*  lesmpkeys_getLocalCSRK(void);

void    lesmpkeys_setPeerCSRK(UINT8 *csrk);
UINT8*  lesmpkeys_getPeerCSRK(void);

void    lesmpkeys_setIR(UINT8 *ir);
void    lesmpkeys_setER(UINT8 *er);
void    lesmpkeys_setIRK(UINT8 *ir);

UINT8  *lesmpkeys_getPeerAdr(void);
UINT8   lesmpkeys_getPeerAdrType(void);

void    lesmpkeys_setPeerAdrAndType(UINT8 *adr, UINT8 type);

void    lesmpkeys_setPeerEDIVAndRand(UINT8 *adr, UINT16 ediv);
UINT16  lesmpkeys_getPeerEDIV(void);
UINT8  *lesmpkeys_getPeerRand(void);

UINT32  lesmpkeys_getPeerCounter(void);
void    lesmpkeys_setPeerCounter(UINT32 cnt);
void    lesmpkeys_incPeerCounter(void);

#if ADDL_LESMPKEYS_API_SUPPORTED
void   lesmpkeys_removeAllBondingInfo(void);
int    lesmpkeys_totalNumOfBondedDevice(void);
#endif
#ifdef __cplusplus
}
#endif

#endif // end of #ifndef _LESMPKEYS_H_
