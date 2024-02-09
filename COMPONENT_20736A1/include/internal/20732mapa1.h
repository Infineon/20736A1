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

#define sr_ptu_status_adr5   0x00360088
#define sr_ptu_status5       (*(volatile unsigned int *)sr_ptu_status_adr5)
#define sr_ptu_en_adr5       0x003600ac
#define sr_ptu_en5           (*(volatile unsigned int *)sr_ptu_en_adr5)
#define dc_ptu_uart2_lcr_adr 0x0036049c
#define dc_ptu_uart2_lcr     (*(volatile unsigned int *)dc_ptu_uart2_lcr_adr)
#define dc_ptu_uart2_rfl_adr 0x003604ac
#define dc_ptu_uart2_rfl     (*(volatile unsigned int *)dc_ptu_uart2_rfl_adr)
#define spiffy_cfg_adr       0x00360600
#define spiffy_cfg           (*(volatile unsigned int *)spiffy_cfg_adr)
#define spiffy2_cfg_adr      0x00361000
#define spiffy2_cfg          (*(volatile unsigned int *)spiffy2_cfg_adr)
#define mia_adc_intf_ctl_adr 0x00390044
#define mia_adc_intf_ctl     (*(volatile unsigned int *)mia_adc_intf_ctl_adr)
