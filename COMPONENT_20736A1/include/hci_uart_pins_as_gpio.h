
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
********************************************************************************
*
* File Name: hci_uart_pins_as_gpio.h
*
* Abstract:  Allows the application to reconfigure HCI UART pins as regular GPIOs.
*            These GPIOs have a max drive strength of 1mA @ Vio = 1.8V and
*            2mA @ Vio 3.2V.
*
*******************************************************************************/
#include "types.h"


#ifndef _HCI_UART_PINS_AS_GPIO_H_
#define _HCI_UART_PINS_AS_GPIO_H_

// The port to use.
enum
{
    GPIO_PORT_CORE = 0
};

enum
{
    GPIO_PIN_CORE_0 = 0,
    GPIO_PIN_CORE_1 = 1,
    GPIO_PIN_CORE_2 = 2,
    GPIO_PIN_CORE_HCI_TX = GPIO_PIN_CORE_2,
    GPIO_PIN_CORE_3 = 3,
    GPIO_PIN_CORE_HCI_RX = GPIO_PIN_CORE_3,
};

/// Reconfigures the given pin on the core port as a GPIO.
/// \param port Always has to be GPIO_PORT_CORE.
/// \param pin  The pin to reconfigure.
void hci_uart_pins_as_gpio_reconfig_as_gpio(UINT8 port, UINT8 pin);

/// Enable wake on the HCI UART RXd pin.
/// \param wakeOnHigh When 1, active high, when 0, active low.
void hci_uart_pins_as_gpio_enableWakeOnRxd(UINT8 wakeOnHigh);

/// Configure GPIO for input/output.
/// \param port Always has to be GPIO_PORT_CORE.
/// \param pin The pin to reconfigure.
/// \param config The configuration of the GPIO. GPIO_OUTPUT_ENABLE for output, GPIO_INPUT_ENABLE
///               for input.
/// \param output The output level when GPIO_OUTPUT_ENABLE is configured. 1 for HIGH, 0 for LOW.
void hci_uart_pins_as_gpio_configurePin(UINT8 port, UINT8 pin, UINT16 config, UINT8 output);

/// Set the output level for an already output enabled GPIO.
/// \param port Always has to be GPIO_PORT_CORE.
/// \param pin  The pin to set output for.
/// \param output The output level when GPIO_OUTPUT_ENABLE is configured. 1 for HIGH, 0 for LOW.
void hci_uart_pins_as_gpio_setPinOutput(UINT8 port, UINT8 pin, UINT8 output);

/// Get the input level for the GPIO.
/// \param port Always has to be GPIO_PORT_CORE.
/// \param pin The pin to read.
UINT8 hci_uart_pins_as_gpio_getPinInput(UINT8 port, UINT8 pin);


#endif   // _HCI_UART_PINS_AS_GPIO_H_
