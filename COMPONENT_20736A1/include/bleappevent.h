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
*
* \file   bleappevent.h
* \brief
*
*  This defines the event interface for the application.
*  An application can serialize callback functions to the app thread
*  from any other thread.
*
*  \author Arvind Sridharan
*  \date   2011-05-05
*******************************************************************************/
#ifndef _BLE_APP_EVENT_H_
#define _BLE_APP_EVENT_H_

#include "cfa.h"
#include "assertpanic.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

/// Request the serialization framework to free the context buffer using cfa_mm_Free()
/// because it was allocated earlier using cfa_mm_Alloc()
#define BLE_APP_EVENT_FREE_BUFFER 42

/// No action needs to be taken by the framework to free the context buffer.
#define BLE_APP_EVENT_NO_ACTION   43

/// The serialization event flag. Internal.
#define BLE_APP_EVENT_CSA_SERIALIZE    (0x1ul << 31)

/// Serialization module initialization function. Invoked by the firmware before the
/// application is initialized. No need to be invoked separately.
void bleappevt_initSerialization(void);

/// Serialize a function call in the application thread context.
/// \param fn Pointer to a function that will be invoked in the application
///           thread context.
/// \param data Pointer to a context meaningful to the callback. This will be passed
///           back to the callback when fn is invoked.
BOOL32 bleappevt_serialize(INT32 (*fn)(void*), void* data);

/// Serialization event handler. Internal to the serialization framewrok.
void bleappevt_serializationHandler(void);




#ifdef __cplusplus
}
#endif
#endif
