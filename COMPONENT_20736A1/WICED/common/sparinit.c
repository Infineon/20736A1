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



/** @file
 *
 */
#include "sparcommon.h"
#include "bleapp.h"
#include "bleappcfa.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

extern void (*bleapp_pre_init)(void);
extern void application_init( void );

/******************************************************
 *               Function Definitions
 ******************************************************/

////////////////////////////////////////////////////////////////////////////////
/// Spar entry function called early during initialization. This function or
/// or any of the called functions cannot allocate memory or create new objects.
/// This function may only be used to initialize other function pointers/register
/// a new app create function/initialize global data.
////////////////////////////////////////////////////////////////////////////////

#ifndef __GNUC__

#pragma arm section code = "spar_setup"

////////////////////////////////////////////////////////////////////////////////
/// Spar entry function called early during initialization. This function or
/// or any of the called functions cannot allocate memory or create new objects.
/// This function may only be used to initialize other function pointers/register
/// a new app create function/initialize global data.
////////////////////////////////////////////////////////////////////////////////
// ATTRIBUTE((section(".setup")))
void application_setup(void)
{
    // Initialize Spar here.
    //extern UINT8 *bleapp_sram_addr;
    //bleapp_sram_addr = (UINT8 *)0x200000;
    //bleapp_pre_init = application_init;
   // BLE_APP_ENABLE_TRACING_ON_HCI_UART();

#ifdef RAMBUFENABLE
    {
        extern UINT8 bleapp_rambuf_enable;
        extern UINT16 cfa_ConfigRambuf_AllocatedLengthInBytes;

        bleapp_rambuf_enable = 1;
        cfa_ConfigRambuf_AllocatedLengthInBytes = 180; //40+3 + 8+3 + 120+3 +3
    }
#endif
}

#pragma arm section code

#else

ATTRIBUTE((section(".spar_setup")))
////////////////////////////////////////////////////////////////////////////////
/// Spar entry function called early during initialization. This function or
/// or any of the called functions cannot allocate memory or create new objects.
/// This function may only be used to initialize other function pointers/register
/// a new app create function/initialize global data.
////////////////////////////////////////////////////////////////////////////////
void SPAR_APP_SETUP(void)
{
    // Initialize Spar here.
    // Typically, replace the default createInstance method of a singleton/multiton/factory
    // with a newCreateInstance etc
    bleappcfa_set_app_uart_coexistence(TRUE); // this indicates to the firmware that app and uart HCI can coexist
    bleapp_pre_init = application_init;

    // these macros become enabled when ENABLE_DEBUG=1 is defined in the application makefile
    // the macros are located here in common code for all 20736-based applications
    // they can be cut and pasted to other locations in the application code
    // a good location is after GPIO initialization and prior to real-time communication
    SETUP_APP_FOR_DEBUG_IF_DEBUG_ENABLED();
    BUSY_WAIT_TILL_MANUAL_CONTINUE_IF_DEBUG_ENABLED();
}
#endif
