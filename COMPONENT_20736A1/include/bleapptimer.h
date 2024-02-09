#ifndef _BLEAPPTIMER_H_
#define _BLEAPPTIMER_H_


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
* File Name: bleapp_timer.h
*
* Abstract: This is the header file for a bleapp timer. The source of the time
*           is cominig from cfa timer.
*
* Functions:
*
*******************************************************************************/


#include "cfa.h"

#ifdef __cplusplus
extern "C" {
#endif


#ifdef _WIN32
#include <pshpack1.h>
#endif
// GHS syntax.
#pragma pack(1)

//////////////////////////////////////////////////////////////////////////////
//                      public data type definition.
//////////////////////////////////////////////////////////////////////////////
typedef void (*BLEAPP_TIMER_CB)(UINT32 arg);
typedef PACKED struct
{
    BLEAPP_TIMER_CB cb;    // this is a call back
    INT16           curTicks;      // 1 second source tick will give a max of 4 hrs.
    INT16           originalTicks; // 1 second source tick will give a max of 4 hrs.
    UINT32          arg;           // client argument to pass back.
} BLEAPPTIMER_BLK;



#define BLEAPPTIMER_INVALID_TIMER_ID                             (-1)



#ifdef _WIN32
#include <poppack.h>
#endif
// GHS syntax.
#pragma pack()

//////////////////////////////////////////////////////////////////////////////
//                      public interface declaration
//////////////////////////////////////////////////////////////////////////////

void bleapptimer_init(void);

// negative number means out of timer.
int bleapptimer_allocateTimer(void);

// this will start Basic stuffs.
void bleapptimer_startTimer(INT32 id, BLEAPP_TIMER_CB cb,INT16 timeOut, UINT16 arg);

// This function will return the Id of the timer. Negative number means out
// of resource.
int bleapptimer_startAppTimer(BLEAPP_TIMER_CB cb,INT16 timeOut, UINT16 arg);

void bleapptimer_refreshAppTimer( INT32 id );

// This interface stops the upper level timers.
void bleapptimer_stopAppTimer( INT32 id );

// This function will start a fine resolution timer. It should be used with care.
// The timeout is in ms unit.
void bleapptimer_startFineTimer( BLEAPP_TIMER_CB cb, INT16 timeOut);
void bleapptimer_stopFineTimer( void );

// This timer is for alert
void bleapptimer_startLEDTimer( BLEAPP_TIMER_CB cb, INT16 timeOut );
void bleapptimer_stopLEDTimer( void );
void bleapptimer_startBUZTimer( BLEAPP_TIMER_CB cb, INT16 timeOut );
void bleapptimer_stopBUZTimer( void );

#ifdef __cplusplus
}
#endif



#endif // end of #ifndef _BLEAPPTIMER_H_
