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
*/

/** @file
*
* Define functions to access PWM peripheral
*/
#ifndef __PWM__H__
#define __PWM__H__

#include "types.h"
#include "gpiodriver.h"

/** \addtogroup  PWM
* \ingroup HardwareDrivers
*/
/*! @{ */
/**
* Defines the standard PWM driver.
* The PWM uses the ACLK as a reference clock and the counts are
* incremented at the ACLK edges starting at init value. When the
* count reaches toggle value, the PWM channel output is inverted
* and when the count reaches 0x3FF, the output is inverted again
* and the count wraps around to init value and so on. When a 50%
* duty cycle is called for, ensure that init value is the same
* distance from toggle value as toggle value is from 0x3FF.
*
* The 4 PWM channels are output to GPIOs P26, P27, P28 and P29. Once the PWM
* is configured, the corresponding GPIO must also be output enabled using
* gpio_configurePin.
*/

/// PWM HW block has 4 PWMs Channels(10 bit)
/// This macros enable to switch from Pwm to its mask value.
/// These mask values are used in enable/disable case.
#define pwmIdToMask(id)           (1<<id)

/// PWM HW block has 4 PWM channels each with its own 10 bit counter.
/// The first PWM id is PWM0;
enum
{
    PWM0  = 0,
    PWM1  = 1,
    PWM2  = 2,
    PWM3  = 3,
    MAX_PWMS = 4

};
/// Clock used for PWM. When LHL_CLK is set, 128 KHz is used. When PMU_CLK is set, 1 MHz or 8 MHz.
typedef enum
{
	/// The 128KHz internal reference clock, continues to run in sleep.
	/// Max possible PWM frequency is ~64KHz.
    LHL_CLK,

    /// The programmable clock derived from aclk, does not run in sleep.
    /// Max possible PWM frequency is ~12MHz.
    PMU_CLK
} PwmClockType;

enum
{
    /// PWM Channel Mask.
    PWM_CHANNEL_MASK        =   0x0F
};

enum
{
	/// PWM0 Output enable on P26
	PWM0_OUTPUT_ENABLE_P26  = GPIO_OUTPUT_ENABLE,

	/// PWM1 Output enable on P27
	PWM1_OUTPUT_ENABLE_P27  = GPIO_OUTPUT_ENABLE,

	/// PWM2 Output enable pn P28
	PWM2_OUTPUT_ENABLE_P28  = GPIO_OUTPUT_ENABLE,

	/// PWM2 Output enable on P6
	PWM2_OUTPUT_ENABLE_P6   = (GPIO_OUTPUT_ENABLE | (1 << 4)),

	/// PWM2 Output enable on P14
	PWM2_OUTPUT_ENABLE_P14  = (2 << 4),

	/// PWM3 Output enable on P29
	PWM3_OUTPUT_ENABLE_P29  = GPIO_OUTPUT_ENABLE,

	/// PWM3 Output enable on P13
	PWM3_OUTPUT_ENABLE_P13  = (2 << 4)
};

/// Start a PWM.
/// \param id - the PWD channel.
/// \param   clk         - which clockspeed to use.
/// \param   toggleCount - value on which a toggle happens.
/// \param   InitCount   - initial value for the counter.
/// \return whether the start is successful.
BOOL32 pwm_start( UINT8 id, PwmClockType clk, UINT32 toggleCount, UINT32 InitCount );

///   Transitions to another set of pulse modulation values.
/// \param id - the PWD channel.
/// \param   toggleCount - value on which a toggle happens.
/// \param   InitCount  - initial value for the counter.
/// \return whether the transition is successful.
BOOL32 pwm_transitionToSubstituteValues(UINT8 id, UINT32 toggleCount, UINT32 InitCount );

///  Alternate between another set of pulse modulation values and current values.
/// \param id - the PWD channel.
/// \param   clk         - which clockspeed to use.
/// \param   toggleCount - value on which a toggle happens.
/// \param   phaseCount  - initial value for the counter.
/// \param   invert      - whether to invert the signal.
/// \return whether the operation is successful.
BOOL32 pwm_startWithAlternateValues( UINT8 id, PwmClockType clk,UINT32 toggleCount, UINT32 InitCount, BOOL32 invert );

/// Get the toggleCount of the PWM channel.
/// \param id - the PWM channel.
///  \return  - the value at which the PWM is going to toggle.
UINT32 pwm_getToggleCount(UINT8 id);

/// Get the Init Value of the PWM channel.
/// \param id - the PWD channel.
/// \return   - initvalue or the Init value of the PWM.
UINT32 pwm_getInitValue(UINT8 id);

/// Set toggleCount. Not glitch free.
/// \param id - the PWD channel.
/// \param toggleCount - value on which a toggle happens.
/// \return whether setToggleCount was successful.
BOOL32 pwm_setToggleCount(UINT8 id, UINT32 toggleCount);

/// Set if output should be inverted or not. This is not glitch free.
/// \param id - the PWD channel.
/// \param invert  Whether to invert the output or not. When TRUE, output will be inverted.
void pwm_setInversion(UINT8 id, BOOL32 invert);

/// Set the values of the given PWM channel.
/// \param id - the PWD channel.
/// \param toggleCount - value on which a toggle happens.
/// \param InitCount   - initial value for the counter.
/// \param clk         - which clockspeed to use.
/// \param invert      - if invert bit is on.
/// \return TRUE if the values are set; else false.
BOOL32 pwm_setValues ( UINT8 id, UINT32 toggleCount, UINT32 initCount, PwmClockType clk, BOOL32 invert);

/// Set the clock.
/// \param id - the PWD channel.
/// \param clk  - which clockspeed to use.
void pwm_setClock(UINT8 id, PwmClockType clk);

///   Set the Initial Value of the Counter.
/// \param id - the PWD channel.
/// \param InitCount   - The initial count to use for the given channel.
void pwm_setInitValue( UINT8 id, UINT32 InitCount );

/// Reset the internal counter. Internal, not for application use.
void pwm_resetInternalCount(UINT32 mask);

/// Enable the given PWM channels.
/// \param mask - the PWD channels to enable.
void pwm_enableChannel(UINT32 mask);

/// Disable the given PWM channels.
/// \param mask - the PWD channels to disable.
void pwm_disableChannel(UINT32 mask);

/// Reset the PWM channels.
/// \param mask - the PWD channels to enable/disable.
/// \param resetEnable - When TRUE, channels will be disabled, when FALSE, channels will be enabled.
void pwm_setReset(UINT32 mask, BOOL32 resetEnable);
/* @} */

#endif
