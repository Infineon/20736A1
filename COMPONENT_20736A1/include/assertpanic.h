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

/*
********************************************************************
*    File Name: assertpanic.h
*
*    Abstract: Fatal assert definition
*
* /!\ The software sometimes detects error conditions which make it
* impossible or very problematic to proceed in any meaningful way.
* An example would be a memory allocation failure. In such situations
* all but the most complex of programs abort. This file contains
* a fatal assert definition for use in such situations.
*
********************************************************************
*/

#ifndef __ASSERT_PANIC_H__
#define __ASSERT_PANIC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* We *DO* *NOT* *WANT* to pass __LINE__ in ASSERT_PANICs in ROM.
 *
 * When you pass __LINE__, and you do a "no-op" change like add a
 * comment to a source file, the __LINE__ for a given ASSERT_PANIC
 * changes. That causes a change to the taped-out image. But it's very
 * useful to be able to add labels, add things under #ifdef,
 * etc. without changing the actual bits when compiled under the right
 * options. So, we pass 0 for the line number. assert_fail() will then
 * use the LR instead. */

#ifdef FPGA_BD_2045

void assert_fail( char*   file,   INT32  line , UINT32 status, char* fatal);
#ifdef ENABLE_ASSERT
#define	ASSERT_PANIC(exp, errCode, errStr)   if(!(exp)) assert_fail( __FILE__, __LINE__, (UINT32)(errCode), (errStr))
#else
#define	ASSERT_PANIC(exp, errCode, errStr)   if(!(exp)) assert_fail( NULL, 0, (UINT32)(errCode), NULL)
#endif

#else // ASIC

void assert_fail( char*   file,   INT32  line , UINT32 status);
#ifdef ENABLE_ASSERT
#define	ASSERT_PANIC(exp, errCode, errStr)   if(!(exp)) assert_fail( __FILE__, __LINE__, (UINT32)(errCode))
#else
#define	ASSERT_PANIC(exp, errCode, errStr)   if(!(exp)) assert_fail( NULL, 0, (UINT32)(errCode))
#endif

#endif

#ifdef __cplusplus
}
#endif

#endif // __ASSERT_PANIC_H__
