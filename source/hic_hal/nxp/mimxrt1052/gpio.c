/**
 * @file    gpio.c
 * @brief   GPIO handling for LPC55xx
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * Copyright (c) 2016-2017 NXP
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "fsl_device_registers.h"
#include "DAP_config.h"
#include "gpio.h"
#include "daplink.h"
#include "hic_init.h"
#include "fsl_clock.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"

__WEAK void board_gpio_init(void)
{
    // Nothing by default
}

void gpio_init(void)
{
    // Enable hardfault on unaligned access for the interface only.
    // If this is done in the bootloader than then it might (will) break
    // older application firmware or firmware from 3rd party vendors.
#if defined(DAPLINK_IF)
    SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif

    // Ensure clocks are enabled.
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    // Configure pins.
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_09_GPIO1_IO09, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_13_GPIO1_IO29, 0U);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_09_GPIO1_IO09, 0x10B0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_13_GPIO1_IO29, 0x10B0U);

    // Set RESET to input
    PIN_PIO_PORT->DR_SET = PIN_RESET_MASK;

    // Turn off LED.
    LED_A_PORT->DR_SET = LED_A_MASK;

    // Set LED to output.
    LED_A_PORT->GDIR |= LED_A_MASK;

    // Turn on LED.
    LED_A_PORT->DR_CLEAR = LED_A_MASK;

    board_gpio_init();
}

void gpio_set_board_power(bool powerEnabled)
{
    // No target power control in this circuit.
}

__WEAK void gpio_set_leds(uint32_t leds, gpio_led_state_t state)
{
    // LED is active low, so set to inverse of the enum value.
    if (leds & (LED_T_CONNECTED | LED_T_HID | LED_T_CDC | LED_T_MSC)) {
        if (state == GPIO_LED_ON) {
            LED_A_PORT->DR_CLEAR = LED_A_MASK;
        } else {
            LED_A_PORT->DR_SET = LED_A_MASK;
        }
    }
}

void gpio_set_hid_led(gpio_led_state_t state)
{
    gpio_set_leds(LED_T_HID, state);
}

void gpio_set_cdc_led(gpio_led_state_t state)
{
    gpio_set_leds(LED_T_CDC, state);
}

void gpio_set_msc_led(gpio_led_state_t state)
{
    gpio_set_leds(LED_T_MSC, state);
}

__WEAK uint8_t gpio_get_reset_btn_no_fwrd(void)
{
    return PIN_nRESET_IN() ? 0 : 1;
}

__WEAK uint8_t gpio_get_reset_btn_fwrd(void)
{
    return 0;
}
