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
* Controller Manager (BLECM).
*
*/
#ifndef _BLECM_H_
#define _BLECM_H_

#include "cfa.h" // Get the Core Firmware API interfaces.
#include "bleapptimer.h" //
#include "dbguart.h" // debug uart.
#include "bleapp.h"

#ifdef __cplusplus
extern "C" {
#endif


/*****************************************************************************/
/** @defgroup blecm       Controller Manager
 *
 *  Bluetooth LE Controller Manager functions
 */
/*****************************************************************************/


//////////////////////////////////////////////////////////////////////////////
//                      public data type definition.
//////////////////////////////////////////////////////////////////////////////
/// Types of some of the callbacks.
typedef void(*BLECM_NO_PARAM_FUNC)(void);
typedef INT32(*BLECM_FUNC_WITH_PARAM)(void *);



// Modules that can be activated.
#define BLECM_MODULE_SMP                                0x0
#define BLECM_MODULE_ATT                                0x1
#define BLECM_MODULE_MAX                                (BLECM_MODULE_ATT +1)

// This enum defined
typedef enum
{
    BLECM_APP_EVT_START_UP,         // 0x0
    BLECM_APP_EVT_LINK_UP,          // 0x1 this is the radio link up.
    BLECM_APP_EVT_LINK_DOWN,        // 0x2 this is the raio link down.
    BLECM_APP_EVT_ADV_TIMEOUT,      // 0x3 this is the adv stop based on timeout
    BLECM_APP_EVT_ENTERING_HIDOFF,  // 0x4 this is to indicate that device is preparing to enter hid-ff
    BLECM_APP_EVT_ABORTING_HIDOFF,  // 0x5 this is to indicate that the device had to abort entering hid-off
    BLECM_APP_EVT_MAX               // this is only used to indicate the size.
} BLECM_APP_EVT_ENUM;

#define BLECM_MAX_CFA_TIMERS                    0x4

#define BLECM_APP_NORMAL_TIMER_ID               0x0
#define BLECM_APP_FINE_RESOLUTION_TIMER_ID      0x1
#define BLECM_APP_LED_TIMER_ID                  0x2
#define BLECM_APP_BUZ_TIMER_ID                  0x3

#ifdef _WIN32
#include <pshpack1.h>
#endif
// GHS syntax.
#pragma pack(1)
/// Defines a structure for use in setting up filter accept lists.
typedef PACKED struct
{
    /// BD_ADDR of the peer to add to/remove from filter accept list.
    UINT8 addr[6];

    /// Type of the peer address - public or random.
    UINT8 type;
}  BLECM_SELECT_ADDR;

/// Internal connection mux data structure.
typedef PACKED struct
{
    UINT16 con_handle;
    UINT16 db_size;
    void *p_db;
    void *dev_pinfo;
    void *smp_pinfo;
} BLECM_CON_MUX_INFO;

#ifdef _WIN32
#include <poppack.h>
#endif
// GHS syntax.
#pragma pack()


//////////////////////////////////////////////////////////////////////////////
//                      public interface declaration
//////////////////////////////////////////////////////////////////////////////
/**
* \brief BLECM Initialization
* \ingroup blecm
*
* \details This function performs BLECM variables initialization.  The function is
* called once during booting up.  Application typically does not need to call
* this function.
*
*/
void blecm_init(void);

/**
* \brief BLECM Start
* \ingroup blecm
*
* \details This function starts BLECM.  The function is called once during booting
* up.  Application typically does not need to call this function.
*
*/
void blecm_start(void);

/**
* \brief Enable/Disable CFA filter
* \ingroup blecm
*
* \details This function enables/disables CFA filter.  When CFA filter is set, all HCI
* commands, events and data are passed to the CFA/profile/application.  When CFA filter is
* not set, all HCI commands, events and data are passed to the transport (UART).
*
* \param enable If 0, CFA filter is not set. if 1, CFA filter is set.
*
*/
void blecm_setFilterEnable(UINT8 enable);

/**
* \brief Reading current CFA filter value
* \ingroup blecm
*
* \details This function returns current CFA filter state.  When CFA filter is set, all HCI
* commands, events and data are passed to the CFA/profile/application.  When CFA filter is
* not set, all HCI commands, events and data are passed to the transport (UART).
*
* \return enable If 0, CFA filter is not set. if 1, CFA filter is set.
*
*/
UINT8 blecm_getFilterEnable(void);


/**
* \brief Register Event Handler
* \ingroup blecm
*
* \details This function should be used before blecm_start is called. This function
* register the start up function for app.  Registered callback functions are called
* whenever related event happens.
*
* \param event_number BLECM_APP_EVT_ENUM
*   \li BLECM_APP_EVT_START_UP        - callback will be called during startup
*   \li BLECM_APP_EVT_LINK_UP         - callback will be called when the radio link comes up.
*   \li BLECM_APP_EVT_LINK_DOWN       - callback will be called radio link goes down.
*   \li BLECM_APP_EVT_ADV_TIMEOUT     - callback will be called when adv stop based on timeout
*   \li BLECM_APP_EVT_ENTERING_HIDOFF - callback will be called to indicate device is preparing to enter hid-ff
*   \li BLECM_APP_EVT_ABORTING_HIDOFF - callback will be called to indicate that the device had to abort entering hid-off
* \param function Callback function that is called whenever event happens
*
*/
void blecm_regAppEvtHandler(BLECM_APP_EVT_ENUM ,BLECM_NO_PARAM_FUNC func);

/**
* \brief Register for ACL Data receive
* \ingroup blecm
*
* \details This function will register a ACL Rx data handler.
*
* \param function Callback function that is called whenever Rx data packet is received
*
*/
void blecm_regAclRxHandler(void *);

/**
* \brief Register for Encryption Changed event
* \ingroup blecm
*
* \details This function will register a callback handler for
* Encryption changed Event.  Application can use this indication if it needs
* to send a packet over encrypted link.
*
* \param function Callback function that is called whenever encryption is changed.
*
*/
void blecm_regEncryptionChangedHandler(void *);

void blecm_activateModule( INT32 moduleIdx, BLECM_FUNC_WITH_PARAM handler);
void blecm_clearActivateModule( INT32 moduleIdx);


/**
* \brief Starts a software timer.
* \ingroup blecm
*
* \details Starts a low level software timer.
*
* \param id ID of the software timer.
* \param appCb Application callback function.
* \param ticks The interval of the timer in OS ticks (12.5mS units).
*
*/
void blecm_startTimerSource(INT32 id, BLECM_FUNC_WITH_PARAM appCb, UINT32 ticks);

/**
* \brief Stopps a software timer.
* \ingroup blecm
*
* \details Stops a low level software timer.
*
* \param id ID of the software timer.
*
*/
void blecm_stopTimerSource( INT32 id );

/**
* \brief Restarts the connection idle timer
* \ingroup blecm
*
* \details When a connection is idle (withou data in either direction),
*          the app may choose to disconnect after while of inactivity.
*          This restarts the idle timer if enabled.
*
*/
void blecm_refreshConnIdletimer(void);

/**
* \brief Starts the connection idle timer
* \ingroup blecm
*
* \details When a connection is idle (withou data in either direction),
*          the app may choose to disconnect after while of inactivity.
*          This starts the idle timer if enabled.
*
* \param cb The callback function that will be invoked when the timer expires.
*/
void blecm_startConnIdleTimer(BLEAPP_TIMER_CB cb) ;

/**
* \brief Stops the connection idle timer
* \ingroup blecm
*
* \details When a connection is idle (withou data in either direction),
*          the app may choose to disconnect after while of inactivity.
*          This Stops the idle timer if enabled.
*
*/
void blecm_stopConnIdleTimer(void);

// Internal function used to check if addres resolution needs to be performed.
int  blecm_needToResolveRandomAddress( void );

/**
* \brief Enable address resolution.
* \ingroup blecm
*
* \details Enables peripheral address resolution.
*
*/
void blecm_enableRandomAddressResolution( void );

/**
* \brief Disable address resolution.
* \ingroup blecm
*
* \details Disable peripheral address resolution.
*
*/
void blecm_disableRandomAddressResolution( void );

/**
* \brief Stores random public address.
* \ingroup blecm
*
* \details Allows the app to provide a random public address for storage in internal IRK.
*
* \param rpa The random public address to store.
* \param idx The index at which to store.
*/
void blecm_storeRPA(UINT8 *rpa, INT32 idx);

/**
* \brief Looks up the given random public address in the key store.
* \ingroup blecm
*
* \details Looks up the given random public address in the key store.
*
* \param type The type of the address.
* \param rpa The Bluetooth address to look up for.
* \return Index of the match or -1 if not found.
*/
int  blecm_lookupAddress(INT32 type, UINT32 *rpa);

/**
* \brief Lookup and resolve address.
* \ingroup blecm
*
* \details Looks up the given random public address in the key store and if
*          not found, resolves it and adds to the key store.
*
* \param type The type of the address.
* \param rpa The Bluetooth address to look up for.
* \return Index of the match when added; -1 when unable to resolve.
*/
int  blecm_resolveRPA(INT32 type, UINT32 *rpa);



// Internal tracing functions.
void blecm_log(char * );
void blecm_log1(char *str,INT32 val );
void blecm_logBytes(char *str,UINT8 *data,INT32 len);
void blecm_log_byteArray(UINT8 *, INT32 len);
void blecm_log_6(char *, INT32, INT32, INT32, INT32,INT32,INT32);



//////////////////////////////////////////////////////////////////////////////
/////////////////              HCI Commands                  /////////////////
//////////////////////////////////////////////////////////////////////////////
// These map directly to HCI commands in section 7 in the Bluetooth spec.

// LE_Set_Advertise_Enable
void blecm_setAdvEnable(INT32 enableFlag);

// LE_Set_Advertising_Parameters
void blecm_setAdvParam(INT32 interval, INT32 advType, INT32 advAdrType,
        INT32 advChanMap, INT32 advFilterPolicy ,INT32 initAdrType ,UINT8 *initAdr);

// HCI_Reset. Never to be used by the app.
void blecm_hciReset(void);

// Read_BD_ADDR
void blecm_hciReadBdAddr(void);

// LE_Rand
void blecm_getRand(void);

// LE_Start_Encryption
void blecm_hciEncrypt(UINT8 *key, UINT8 *text);

// LE_Long_Term_Key_Requested_Reply
void blecm_hciLTKReply(UINT16 handle,UINT8 *ltk);

// LE_Long_Term_Key_Requested_Negeive_Reply
void blecm_hciLTKNegReply(UINT16 handle);

// Disconnect
void blecm_disconnect(INT32 errCode );

// LE_Set_Advertising_Data
void blecm_setAdvData(UINT8 *data, INT32 len);

// LE_Set_Scan_Response
void blecm_setScanRspData(UINT8 *data, INT32 len);

/**
* Enable/disable Advertisement during connection
*
* \param enableFlag 1 - Enable, 0 - disable
*/
UINT32 blecm_setAdvDuringConnEnable(int enableFlag);

/**
* Enable scatternet in the controller
*
* \param NONE
*/
void blecm_enablescatternet(void);

/**
* Sets the maximum TX power
*
* \param maxTxPowerDb The max TX power allowed in dB.
*/
void blecm_setTxPowerInConnection(INT8 maxTxPowerDb);

/**
* Ends all LE tests that were started with blecm_StartReveiverTest or blecm_StartTransmitterTest
*
* \param NONE
*/
void blecm_EndTest(void);

/**
* Starts the standard LE receiver test on the given channel
*
* \param channel The channel to use for the test. 0-39.
* \return <ReturnValue> True if test was started successfully, else could not start the test.
*/
UINT8 blecm_StartReveiverTest(UINT8 channel);

/**
* Starts the standard LE transmitter test with the given parameters
*
* \param channel  The channel to use for the test. 0-39.
* \param packetLength The Length test packet. 0-37
* \param packetType   The type of packet to use - The following are supported:
*               {0x00, "Pseudo-Random bit sequence 9"},
*               {0x01, "Pattern of alternating bits '11110000'"},
*               {0x02, "Pattern of alternating bits '10101010'"},
*               {0x03, "Pseudo-Random bit sequence 15 - Optional"},
*               {0x04, "Pattern of All '1' bits - Optional"},
*               {0x05, "Pattern of All '0' bits - Optional"},
*               {0x06, "Pattern of alternating bits '00001111' - Optional"},
*               {0x07, "Pattern of alternating bits '01010101' - Optional"}
* \return <ReturnValue> True if test was started successfully, else could not start the test.
*/
UINT8 blecm_StartTransmitterTest(UINT8 channel, UINT8 packetLength, UINT8 packetType);

/**
* Set advertising parameters and starts advertisements.
* \details Sets the advertisement parameters and starts it.
* \param advType The advertismenet type. See LE_Set_Advertising_Parameters HCI command for acceptable values.
* \param advInterval The advertisement interval in Bluetooth slots (min 32, max 16384).
* \param advChannel The advertisement channel mask (can bitwise OR). 0x01 for ch37; 0x02 for ch38; 0x04 for ch39.
* \param advAdrType The advertiser type (Public/random).
* \param advFilterPolicy The filter policy to use. See Bluetooth spec for acceptable values.
* \param initiatorAdrType The address type of the peer device - valid only when advType is directed advertisement.
* \param initiatorAdr The address of the peer device - valid only when advType is directed advertisement.
*/
void blecm_startAdv(INT32 advType, INT32 advInterval, INT32 advChannel, INT32 advAdrType,
                    INT32 advFilterPolicy, INT32 initiatorAdrType, UINT8* initiatorAdr);


/**
* Gets the number of available transmit buffers in the controller.
*/
INT32 blecm_getAvailableTxBuffers(void);

// Internal function, not for application use.
INT32 blecm_incAvailableTxBuffers(void);
// Internal function, not for application use.
INT32 blecm_decAvailableTxBuffers(void);
// Internal function, not for application use.
void blecm_setAvailableTxBuffers(INT32 val);

// Internal function to check if filter accept list is enabled.
INT32  blecm_needToSelectAddress( void );

/**
* Clears and then adds the given addresses to the filter accept list.
* \param p_select_addr Array of addresses to add to the filter accept list.
* \param num The number of items to add to the filter accept list. Max is 5.
* \return 1 on success; 0 on failure.
*/
INT32 blecm_SelectAddress (BLECM_SELECT_ADDR *p_select_addr, UINT8 num);
// Internal function.
INT32 blecm_SelectTargetAddress (BLECM_SELECT_ADDR *p_select_addr, UINT8 num);

/**
* Enables filter accept list at the stack leyer.
*/
void blecm_enableAddressSelection( void );

/**
* Disables filter accept list at the stack leyer. To fully disable filter accept list, call
* blecm_SelectAddress() with NULL, 0 and then invoke this.
*/
void blecm_disableAddressSelection( void );

/**
* \brief Configure LE scan parameters and start LE scan
* \ingroup blecm
*
* \details This function will start LE Scan with the given parameters. The
* Scan will start shortly after but not immediately after this function call returns.
*
* \param LE_Scan_Type
*       \li HCIULP_PASSIVE_SCAN Passive Scanning. No SCAN_REQ packets shall be sent.
*       \li HCIULP_ACTIVE_SCAN Active scanning. SCAN_REQ packets may be sent.
* \param LE_Scan_Interval Time interval from when the Controller started its last LE scan
* until it begins the subsequent LE scan.
* \param LE_Scan_Window The duration of the LE scan.
* \param Own_Address_Type
*       \li HCIULP_PUBLIC_ADDRESS
*       \li HCIULP_RANDOM_ADDRESS
* \param scanFilterPolicy
*       \li HCIULP_SCAN_FILTER_POLICY_ACCEPT_LIST_NOT_USED
*       \li HCIULP_SCAN_FILTER_POLICY_ACCEPT_LIST_USED
* \param Filter Duplicates
*       \li HCIULP_SCAN_DUPLICATE_FILTER_OFF Duplicate filtering disabled
*       \li HCIULP_SCAN_DUPLICATE_FILTER_ON Duplicate filtering enabled
*
*/
void blecm_startScan(INT32 scanType, INT32 scanInterval, INT32 scanWindow, INT32 scanAdrType,
                     INT32 scanFilterPolicy, INT32 filterDuplicates );
/**
* Set the LE scan parameters.
* \param LE_Scan_Type
*       \li HCIULP_PASSIVE_SCAN Passive Scanning. No SCAN_REQ packets shall be sent.
*       \li HCIULP_ACTIVE_SCAN Active scanning. SCAN_REQ packets may be sent.
* \param LE_Scan_Interval Time interval from when the Controller started its last LE scan
* until it begins the subsequent LE scan.
* \param LE_Scan_Window The duration of the LE scan.
* \param Own_Address_Type
*       \li HCIULP_PUBLIC_ADDRESS
*       \li HCIULP_RANDOM_ADDRESS
* \param scanFilterPolicy
*       \li HCIULP_SCAN_FILTER_POLICY_ACCEPT_LIST_NOT_USED
*       \li HCIULP_SCAN_FILTER_POLICY_ACCEPT_LIST_USED
*/
void blecm_setScanParam(INT32 scanType, INT32 scanInterval, INT32 scanWindow, INT32 scanAdrType,
                        INT32 scanFilterPolicy );
/**
* Enable LE scans.
* \param filterDuplicates
*       \li HCIULP_SCAN_DUPLICATE_FILTER_OFF Duplicate filtering disabled
*       \li HCIULP_SCAN_DUPLICATE_FILTER_ON Duplicate filtering enabled
*/
void blecm_scanOn(INT32 filterDuplicates);

/**
* \brief Configure controller for LE scan
* \ingroup blecm
*
* \details This function issues HCI LE Set Scan Enable Command.  This
* command may be used to start and configure, or stop LE scans.
*
* \param LE_Scan_Enable
*       \li HCIULP_SCAN_MODE_OFF LE scanninig disabled
*       \li HCIULP_SCAN_MODE_ON LE scanninig enabled
* \param Filter Duplicates
*       \li HCIULP_SCAN_DUPLICATE_FILTER_OFF Duplicate filtering disabled
*       \li HCIULP_SCAN_DUPLICATE_FILTER_ON Duplicate filtering enabled
*
*/
void blecm_setScanEnable(INT32 enableFlag, INT32 filterDuplicates );

/**
* \brief Create a Link Layer connection to a connectable advertiser.
* \ingroup blecm
*
* \details This function issues HCI LE Create Connection Command.  Typically
* this command will be used after client application receives advertisement
* from a peripheral device
*
* \param LE_Scan_Interval Time interval from when the Controller started its last LE scan
* until it begins the subsequent LE scan.
* \param LE_Scan_Window The duration of the LE scan.
* \param Initiator_Filter_Policy
* \param scanFilterPolicy
*       \li HCIULP_INITIATOR_FILTER_POLICY_ACCEPT_LIST_NOT_USED
*       \li HCIULP_INITIATOR_FILTER_POLICY_ACCEPT_LIST_USED
* \param Peer_Address_Type
*       \li HCIULP_PUBLIC_ADDRESS
*       \li HCIULP_RANDOM_ADDRESS
* \param ownAddressType
*       \li HCIULP_PUBLIC_ADDRESS
*       \li HCIULP_RANDOM_ADDRESS
* \param PeerAddress Public Device Address or Random Device Address of the device to be connected
* \param connMinInterval Minimum value for the connection event interval
* \param connMaxInterval Maximum value for the connection event interval
* \param connLatency Peripheral latency for the connection in number of connection events
* \param supervisionTimeout Supervision timeout for the LE Link
* \param connMinEventLen Minimum length of connection needed for this LE connection
* \param connMaxEventLen Maximum length of connection needed for this LE connection
*
*/
void blecm_CreateConnection(INT32 scanInterval, INT32 scanWindow, INT32 initiatorFilterPolicy,
                            INT32 peerAddressType, char *peerAddress, INT32 ownAddressType,
                            INT32 connMinInterval, INT32 connMaxInterval, INT32 connLatency,
                            INT32 supervisionTimeout, INT32 connMinEventLen, INT32 connMaxEventLen);
/**
* \brief Cancel attempt to create a Link Layer connection
* \ingroup blecm
*
* \details This function may be issued by the client application to
* send HCI LE Create Connection Cancel Command.
*
*/
void blecm_CreateConnectionCancel(void);

/**
* \brief Connection update request
* \ingroup blecm
*
* \details This function issues HCI LE Connection Update Command.  This
* command may be used in the client application if it needs to update
* connection parameters when connection is already established and
* required parameters are different than ones requested by the peripheral.
* When peripheral device requests some parameters using L2CAP message, stack
* automatically replies and sets controller appropriately and there is no
* need to use this function call.
*
* \param connHandle HCI handle of established connection
* \param connMinInterval Minimum value for the connection event interval
* \param connMaxInterval Maximum value for the connection event interval
* \param connLatency Peripheral latency for the connection in number of connection events
* \param supervisionTimeout Supervision timeout for the LE Link
* \param connMinEventLen Minimum length of connection needed for this LE connection
* \param connMaxEventLen Maximum length of connection needed for this LE connection
*
*/
void blecm_ConnectionUpdate(INT32 connHandle, INT32 connMinInterval, INT32 connMaxInterval,
                            INT32 connLatency, INT32 supervisionTimeout, INT32 connMinEventLen,
                            INT32 connMaxEventLen);

/**
* \brief Register to receive advertisement report
* \ingroup blecm
*
* \details Client application may use this function to receive notification when
* controller receives advertisement from the peripheral device.
*
* \param function Callback function that is called whenever advertisement is received.
* HCIULP_ADV_PACKET_REPORT_WDATA structure with advertisement data is passed as a
* parameter to this function.
*/
void blecm_RegleAdvReportCb(BLECM_FUNC_WITH_PARAM cb);

// Internal function.
void blecm_startEncryption(INT32 Connection_Handle, UINT8 *Random_Number,
                           INT32 Encrypted_Diversifier, UINT8 *Long_Term_Key);


// The stack/app architecture supports only one connection at any point of time.
// Since the stack and app are event driven, a software mux is used to switch the
// app and stack connection context when some event/action needs to be performed on
// different connections. These functions provide the means to switch between different
// connection contexts.

// Internal function used by the stack to check if the connection context needs to be changed.
INT32  blecm_needToConMux( void );

/**
* Enable connection muxing. This needs to be done once by the app in the create function.
*/
void   blecm_enableConMux( void );

/**
* Disable connection muxing. Generally never done.
*/
void   blecm_disableConMux( void );

/**
* Initialize connection mux. This nees to be done once by the app in the create function.
* \param con_num The max number of connections that can be active at any point in time.
*/
void   blecm_ConMuxInit(INT32 con_num);

/**
* Add the connection to the list of known connections we can mux between. Typically done on connection up.
* \param index The index into which to add the connection information.
* \param con_handle The connection handle of this connection. This handle
*                    is used to look up the connection info structure.
* \param db_size  The size of the GATT DB to be associated with the connection.
* \param p_db The pointer to the GATT DB to be associated with this connection.
* \param dev_pinfo Device information in EMCONINFO_DEVINFO for this connection.
* \param smp_pinfo The SMP information from LESMP_INFO for this connection.
*/
void   blecm_AddConMux(INT32 index, UINT16 con_handle, UINT16 db_size, void *p_db,
                       void *dev_pinfo, void *smp_pinfo);

/**
* Delete the connection mux/context at the given index. Typically done on connection down.
* \param index The index of the connection information to delete.
*/
void   blecm_DelConMux(int index);

/**
* Find a free connection mux/context slot.
* \return Index of the first unused connection info structure or -1 when none are free.
*/
INT32  blecm_FindFreeConMux(void);

/**
* Look up the connection information structure given a handle.
* \param con_handle Connection handle of the connection to look up for.
* \return Index of the connection info structure or -1 when not found.
*/
INT32  blecm_FindConMux(UINT16 con_handle);

/**
* Get the GATT DB size given a connection handle.
* \param con_handle Connection handle of the connection to look up for.
* \return GATT DB size or 0 when not found.
*/
UINT16 blecm_GetDbSizeConMux(UINT16 con_handle);

/**
* Get the GATT DB  given a connection handle.
* \param con_handle Connection handle of the connection to look up for.
* \return GATT DB or NULL when not found.
*/
void   *blecm_GetDbConMux(UINT16 con_handle);

/**
* Get the device information given a connection handle.
* \param con_handle Connection handle of the connection to look up for.
* \return Device information pointer or NULL when not found.
*/
void   *blecm_GetDevConMux(UINT16 con_handle);

/**
* Get the SMP information given a connection handle.
* \param con_handle Connection handle of the connection to look up for.
* \return SMP information pointer or NULL when not found.
*/
void   *blecm_GetSmpConMux(UINT16 con_handle);

/**
* Reset the connection context.
*/
void   blecm_ResetPtrConMux(void);

/**
* Set all the connection contexts to the one for the given connection.
* This is used by the stack to switch connection contexts.
*/
void   blecm_SetPtrConMux(UINT16 con_handle);

// Internal function to check if 8-bitor 16-bit length is used in the app's GATT DB.
INT32  blecm_needToGATTDB16( void );

/**
* Enable 16-bit length fields in application GATT DB. This needs to be invoked before the first
* access to GATT DB occurs (early in create function). The GATT DB provided in the set_cfg function
* should use 16-bit lengths.
*/
void   blecm_enableGATTDB16( void );

/**
* Disable 16-bit length fields in app GATT DB.
*/
void   blecm_disableGATTDB16( void );

#ifdef APP_POLL_API_SUPPORTED
/**
* Initialize application poll mechanism.
*/
void   blecm_initAppPoll(void);
void   blecm_appPollEnable(void (*clientCallback)(void*, UINT32),
                           UINT32 clientContext,
                           UINT16 defaultPeriod);
void   blecm_appPollDisable(void);
#endif // APP_POLL_API_SUPPORTED

//////////////////////////////////////////////////////////////////////////////
// These MACROS controlls debugging print.
#define BLECM_DBGUART_LOG                                              0x1
#define BLECM_DBGUART_LOG_L2CAP                                        0x2
#define BLECM_DBGUART_LOG_SMP                                          0x4
#define BLECM_DBGUART_LOG_ATT                                          0x8
#define BLECM_DBGUART_LOG_AP                                           0x10

extern UINT32 blecm_configFlag ;
#define BLECM_DBGUART_LOG_ENABLED()  (blecm_configFlag & BLECM_DBGUART_LOG)
#define BLECM_DBGUART_LOG_L2CAP_ENABLED()  \
            (blecm_configFlag & BLECM_DBGUART_LOG_L2CAP)
#define BLECM_DBGUART_LOG_SMP_ENABLED()  \
            (blecm_configFlag & BLECM_DBGUART_LOG_SMP)
#define BLECM_DBGUART_LOG_ATT_ENABLED()  \
            (blecm_configFlag & BLECM_DBGUART_LOG_ATT)
#define BLECM_DBGUART_LOG_AP_ENABLED()  \
            (blecm_configFlag & BLECM_DBGUART_LOG_AP)

#ifdef __cplusplus
}
#endif
#endif // end of #ifndef _BLECM_H_
