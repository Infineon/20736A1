/*******************************************************************
*
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
*
********************************************************************
*
********************************************************************************
*
* File Name: thread_and_mem_mgmt.h
*
* Abstract:  Provides some extra API for memory and application thread management.
*
*******************************************************************************/
#include "types.h"
#include "cfa.h"

#ifndef _THREAD_AND_MEM_MGMT_H_
#define _THREAD_AND_MEM_MGMT_H_
enum
{
    /// General memory pool 0.
    CFA_MM_POOL_0,

    /// General memory pool 1.
    CFA_MM_POOL_1,

    /// General memory pool 2.
    CFA_MM_POOL_2
};

////////////////////////////////////////////////////////////////////////////////
/// Prepares the stack to allow the app to check for stack overflow.
/// For blecm_DidStackOverflow() to be used by the app, this (blecm_StackCheckInit)
/// must be called once in application_create function.
////////////////////////////////////////////////////////////////////////////////
void blecm_StackCheckInit(void);

////////////////////////////////////////////////////////////////////////////////
/// Checks if the application thread stack overflowed at some point.
///
/// \param void
/// \return TRUE if it overflowed; FALSE if no overflow was detected.
////////////////////////////////////////////////////////////////////////////////
UINT8 blecm_DidStackOverflow(void);

////////////////////////////////////////////////////////////////////////////////
/// Sets the stack size for the application thread. This should be invoked only
/// in the context of the APPLICATION_INIT function (or the function that
/// invokes bleapp_set_cfg). Calling this in any other context is undefined.
/// Default is 256 = 1024 byte stack and this is the minimum recommended. Increasing
/// this will reduce the available RAM for the app. If the app needs a 4096 byte stack,
/// invoke like this:
/// blecm_SetApplicationThreadStackSizeInWords(4096/sizeof(unsigned));
/// \param number_of_32bit_words Size of the stack to be allocated in machine words.
////////////////////////////////////////////////////////////////////////////////
void blecm_SetApplicationThreadStackSizeInWords(UINT32 number_of_32bit_words);

////////////////////////////////////////////////////////////////////////////////////////////////////
/// Allows the application to configure the size and number of memory blocks in the general pool.
/// There are only 3 memory pools in the system, 0, 1 and 2. The following is the default
/// configuration of the pools:
/// pool[0].size = 32, pool[0].num = 28
/// pool[1].size = 64, pool[1].num = 6 + max_num_connections (= 4 by default)
/// pool[2].size = 264, pool[2].num = 3
/// If you want pool 2 to be 10 buffers with each being 272 bytes, then invoke it this way:
/// cfa_mm_ConfigureMemoryPool(CFA_MM_POOL_2, 272, 10);
/// Note that this can only be done only in the context of the APPLICATION_INIT function
/// (or the function that invokes bleapp_set_cfg). Calling this in any other context is undefined.
/// Making any of these pools larger than the defaults will reduce the amount of free RAM available
/// to the app. Use with caution.
/// If you are setting max number of connections using blecm_setmaxconnection(), make sure that this
/// function is invoked after setting max number of connections, or the behavior is undefined.
/// \param pool_id Pool ID of the pool being configured. Must be one of CFA_MM_POOL_X.
/// \param block_size Size of (in bytes) each buffer in this pool.
/// \param block_num Number of buffers to allocate in this pool.
/// \return TRUE on success, else failure.
////////////////////////////////////////////////////////////////////////////////////////////////////
UINT32 cfa_mm_ConfigureMemoryPool(UINT32 pool_id, UINT32 block_size, UINT32 );

#endif   // _THREAD_AND_MEM_MGMT_H_
