/*******************************************************************
*
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
********************************************************************
*
*
********************************************************************************
*
* File Name: long_characteristic_support.h
*
* Abstract:  Provides some extra bleprofile_* API missing in 20736/7.
*
*******************************************************************************/
#include "types.h"
#include "cfa.h"

#ifndef _ADDITIONAL_BLEPROFILE_API_H_
#define _ADDITIONAL_BLEPROFILE_API_H_

/// Allows the app to read a characteristic (larger than 23 bytes) from its GATT DB.
/// When using buffer after reading, thye len field will have the actual length read.
/// Note that the allocated buffer needs to be 2 + max size of
/// the characteristic you want to read. For example, if the characteristic you want to
/// read is max 50 bytes long and its handle is 0x1234:
/// #define MY_CHARACTERISTIC_SIZE_MAX     50
/// // Allocate a buffer big enough for my characterestic = MY_CHARACTERISTIC_SIZE_MAX + size of header.
/// UINT16 size_to_allocate = sizeof(UINT8) + sizeof(UINT8) + MY_CHARACTERISTIC_SIZE_MAX;
/// BLEPROFILE_DB_PDU* my_characteristic = cfa_mm_alloc(size_to_allocate);
/// // Assuming allocation succeeded here. You need to check for a NULL return value.
/// if(bleprofile_ReadHandleData(0x1234, my_characteristic, size_to_allocate) == 0)
/// {
///     // Successful.
///     UINT8 characterestic_length = my_characteristic->len;
///     UINT8* characteristic_value_ptr = my_characteristic->pdu;
/// }
/// // Done with buffer, so free it.
/// cfa_mm_Free(my characterestic);
/// \param hdl The handle of the characteristic to read.
/// \param buffer Pointer to an allocated buffer into which to read into.
/// \param pdu_alloc_length Length of the allocated buffer = sizeof(UINT8) + sizeof(UINT8) + MY_CHARACTERISTIC_SIZE_MAX
/// \return 0 for success, else failure code.
int bleprofile_ReadHandleData(UINT16 hdl, BLEPROFILE_DB_PDU* buffer, UINT16 pdu_alloc_length);

/// Allows the app to write a characteristic (larger than 23 bytes) to its GATT DB.
/// If my_characteristic is 100 bytes long:
/// BLEPROFILE_DB_PDU* my_characterestic = cfa_mm_Alloc(sizeof(UINT8) + sizeof(UINT8) + 100);
/// //// Fill up the characteristic pdu value however you see fit. For example:
/// my_characterestic->pdu[99] = 0x42;
/// if(bleprofile_WriteHandleData(0x1234, my_characteresti, sizeof(UINT8) + sizeof(UINT8) + 100))
/// {
///     // All good....
/// }
/// // Now free the buffer
/// cfa_mm_Free(my_characteresti);
/// \param hdl The handle of the characteristic to write to.
/// \param buffer Pointer to the characteristic value.
/// \param length Length of the characteristic to write
/// \return 0 for success, else failure code.
int bleprofile_WriteHandleData(UINT16 hdl, BLEPROFILE_DB_PDU* buffer, UINT16 length);

/// Set the maximum number of prepare write requests this GATT server can take before needing to execute them.
/// The default is 5 (i.e. you can have at most 5 prepare write requests before the execute, the 6th
/// will return an insufficient resources errror). If the memory pool manager limits are hit before
/// this, prepare writes will still fail.
/// \param queue_size Max number of prepare write requests before needing to execute the writes.
void bleprofile_SetMaxQueuedWriteRequests(UINT16 queue_size);


#endif  // _ADDITIONAL_BLEPROFILE_API_H_
