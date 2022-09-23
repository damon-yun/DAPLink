/**
 * @file    hic_init.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2020 Arm Limited, All Rights Reserved
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

#include "hic_init.h"
#include "daplink.h"


//! Setup clocks to run from the FRO HF at 96 MHz.
void sdk_init(void)
{

}

//! - Configure the VBUS pin.
//! - Switch USB1 to device mode.
//! - Turn on 16MHz crystal oscillator.
//! - Switch main clock to PLL0 at 150 MHz.
//! - Ungate USBPHY and USBHS.
//! - Configure the USB PHY and USB1 clocks.
void hic_enable_usb_clocks(void)
{
}

void hic_power_target(void)
{

}

// Override the default weak implementation.
bool flash_is_readable(uint32_t addr, uint32_t length)
{
    return true;
}
