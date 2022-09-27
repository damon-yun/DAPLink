/**
 * @file    jtag_host.h
 * @brief   Host driver for accessing the DAP
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
 * Copyright 2019, Cypress Semiconductor Corporation
 * or a subsidiary of Cypress Semiconductor Corporation.
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

#ifndef JTAG_CM_H
#define JTAG_CM_H

#include "flash_blob.h"
#include "target_family.h"
#ifdef TARGET_MCU_CORTEX_A
#include "debug_ca.h"
#else
#include "debug_cm.h"
#endif


#define JTAG_REG_ADR(a)    (a & 0x0c)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
   JTAG_CONNECT_NORMAL,
   JTAG_CONNECT_UNDER_RESET,
} JTAG_CONNECT_TYPE;

typedef enum {
    JTAG_FLASHALGO_RETURN_BOOL,
    JTAG_FLASHALGO_RETURN_POINTER
} jtag_flash_algo_return_t;

uint8_t jtag_init(void);
uint8_t jtag_off(void);
uint8_t jtag_init_debug(void);
uint8_t jtag_clear_errors(void);
void    jtag_write_ir(uint32_t ir);
uint8_t jtag_read_dp(uint8_t adr, uint32_t *val);
uint8_t jtag_write_dp(uint8_t adr, uint32_t val);
uint8_t jtag_read_ap(uint32_t adr, uint32_t *val);
uint8_t jtag_write_ap(uint32_t adr, uint32_t val);
uint8_t jtag_read_word(uint32_t addr, uint32_t *val);
uint8_t jtag_write_word(uint32_t addr, uint32_t val);
uint8_t jtag_read_byte(uint32_t addr, uint8_t *val);
uint8_t jtag_write_byte(uint32_t addr, uint8_t val);
uint8_t jtag_read_memory(uint32_t address, uint8_t *data, uint32_t size);
uint8_t jtag_write_memory(uint32_t address, uint8_t *data, uint32_t size);
uint8_t jtag_read_core_register(uint32_t n, uint32_t *val);
uint8_t jtag_write_core_register(uint32_t n, uint32_t val);
uint8_t jtag_flash_syscall_exec(const program_syscall_t *sysCallParam, uint32_t entry, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4, jtag_flash_algo_return_t return_type);
uint8_t jtag_set_target_state_hw(target_state_t state);
uint8_t jtag_set_target_state_sw(target_state_t state);
uint8_t jtag_transfer_retry(uint32_t req, uint32_t *data);
void    jtag_set_soft_reset(uint32_t soft_reset_type);
uint8_t SWD2JTAG(void);

#ifdef __cplusplus
}
#endif

#endif
