/**
 * @file    jtag_host.c
 * @brief   Implementation of jtag_host.h
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

#ifndef TARGET_MCU_CORTEX_A
#include "device.h"
#include "cmsis_os2.h"
#include "target_config.h"
#include "DAP_config.h"
#include "DAP.h"
#include "target_family.h"
#include "jtag_host.h"

// Default NVIC and Core debug base addresses
// TODO: Read these addresses from ROM.
#define NVIC_Addr    (0xe000e000)
#define DBG_Addr     (0xe000edf0)

// AP CSW register, base value
#define CSW_VALUE (CSW_RESERVED | CSW_MSTRDBG | CSW_HPROT | CSW_DBGSTAT | CSW_SADDRINC)

#define DCRDR 0xE000EDF8
#define DCRSR 0xE000EDF4
#define DHCSR 0xE000EDF0
#define REGWnR (1 << 16)

#define MAX_SWD_RETRY 100//10
#define MAX_TIMEOUT   1000000  // Timeout for syscalls on target

// Use the CMSIS-Core definition if available.
#if !defined(SCB_AIRCR_PRIGROUP_Pos)
#define SCB_AIRCR_PRIGROUP_Pos              8U                                            /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */
#endif

typedef struct {
    uint32_t select;
    uint32_t csw;
} DAP_STATE;

typedef struct {
    uint32_t r[16];
    uint32_t xpsr;
} DEBUG_STATE;

static uint32_t jtag_ir_post = 0;
static DAP_STATE dap_state;
static uint32_t  soft_reset = SYSRESETREQ;

static uint32_t jtag_get_apsel(uint32_t adr)
{
    uint32_t apsel = target_get_apsel();
    if (!apsel)
        return adr & 0xff000000;
    else
        return apsel;
}

static void __int2array(uint8_t *res, uint32_t data, uint8_t len)
{
    uint8_t i = 0;

    for (i = 0; i < len; i++) {
        res[i] = (data >> 8 * i) & 0xff;
    }
}

uint8_t jtag_transfer_retry(uint32_t req, uint32_t *data)
{
    uint8_t i, ack;

    for (i = 0; i < MAX_SWD_RETRY; i++) {
        ack = JTAG_Transfer(req, data);

        // if ack != WAIT
        if (ack != DAP_TRANSFER_WAIT) {
            return ack;
        }
    }

    return ack;
}

void jtag_set_soft_reset(uint32_t soft_reset_type)
{
    soft_reset = soft_reset_type;
}

uint8_t jtag_init(void)
{
    //TODO - DAP_Setup puts GPIO pins in a hi-z state which can
    //       cause problems on re-init.  This needs to be investigated
    //       and fixed.
    DAP_Setup();
    PORT_JTAG_SETUP();
    return 1;
}

uint8_t jtag_off(void)
{
    PORT_OFF();
    return 1;
}

void jtag_write_ir(uint32_t ir)
{
    if (jtag_ir_post != ir) {
        JTAG_IR(ir);
        jtag_ir_post = ir;
    }
}

uint8_t jtag_clear_errors(void)
{
    if (!jtag_write_dp(DP_ABORT, STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR)) {
        return 0;
    }
    return 1;
}

// Read debug port register.
uint8_t jtag_read_dp(uint8_t adr, uint32_t *val)
{
    uint32_t tmp_in;
    uint8_t tmp_out[4];
    uint8_t ack;
    uint32_t tmp;

    jtag_write_ir(JTAG_DPACC);

    tmp_in = DAP_TRANSFER_RnW | JTAG_REG_ADR(adr);
    // first dummy read
    jtag_transfer_retry(tmp_in, (uint32_t *)tmp_out);
    ack = jtag_transfer_retry(tmp_in, (uint32_t *)tmp_out);
    *val = 0;
    tmp = tmp_out[3];
    *val |= (tmp << 24);
    tmp = tmp_out[2];
    *val |= (tmp << 16);
    tmp = tmp_out[1];
    *val |= (tmp << 8);
    tmp = tmp_out[0];
    *val |= (tmp << 0);

    return (ack == 0x01);
}

// Write debug port register
uint8_t jtag_write_dp(uint8_t adr, uint32_t val)
{
    uint32_t req;
    uint8_t data[4];
    uint8_t ack;

    //check if the right bank is already selected
    if ((adr == DP_SELECT) && (dap_state.select == val)) {
        return 1;
    }

    jtag_write_ir(JTAG_DPACC);

    __int2array(data, val, 4);
    ack = jtag_transfer_retry(JTAG_REG_ADR(adr), (uint32_t *)data);
    if ((ack == DAP_TRANSFER_OK) && (adr == DP_SELECT)) {
        dap_state.select = val;
    }
    return (ack == 0x01);
}

// Read access port register.
uint8_t jtag_read_ap(uint32_t adr, uint32_t *val)
{
    uint32_t tmp_in;
    uint8_t tmp_out[4];
    uint8_t ack;
    uint32_t tmp;

    jtag_write_ir(JTAG_APACC);

    tmp_in = DAP_TRANSFER_RnW | JTAG_REG_ADR(adr);
    // first dummy read
    jtag_transfer_retry(tmp_in, (uint32_t *)tmp_out);
    ack = jtag_transfer_retry(tmp_in, (uint32_t *)tmp_out);

    *val = 0;
    tmp = tmp_out[3];
    *val |= (tmp << 24);
    tmp = tmp_out[2];
    *val |= (tmp << 16);
    tmp = tmp_out[1];
    *val |= (tmp << 8);
    tmp = tmp_out[0];
    *val |= (tmp << 0);

    return (ack == 0x01);
}

// Write access port register
uint8_t jtag_write_ap(uint32_t adr, uint32_t val)
{
    uint8_t data[4];
    uint8_t req, ack;
    uint32_t apsel = jtag_get_apsel(adr);
    uint32_t bank_sel = adr & APBANKSEL;

    if (!jtag_write_dp(DP_SELECT, apsel | bank_sel)) {
        return 0;
    }

    switch (adr) {
        case AP_CSW:
            if (dap_state.csw == val) {
                return 1;
            }

            dap_state.csw = val;
            break;

        default:
            break;
    }

    jtag_write_ir(JTAG_APACC);

    __int2array(data, val, 4);

    if (jtag_transfer_retry(JTAG_REG_ADR(adr), (uint32_t *)data) != 0x01) {
        return 0;
    }
    return 1;
}


// Write 32-bit word aligned values to target memory using address auto-increment.
// size is in bytes.
static uint8_t jtag_write_block(uint32_t address, uint8_t *data, uint32_t size)
{
    uint8_t tmp_in[4], req;
    uint32_t size_in_words;
    uint32_t i, ack;

    if (size == 0) {
        return 0;
    }

    jtag_write_ir(JTAG_APACC);

    size_in_words = size / 4;

    // CSW register
    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32)) {
        return 0;
    }

    // TAR write
    req = JTAG_REG_ADR(AP_TAR);
    __int2array(tmp_in, address, 4);

    if (jtag_transfer_retry(req, (uint32_t *)tmp_in) != 0x01) {
        return 0;
    }

    // DRW write
    req = JTAG_REG_ADR(AP_DRW);

    for (i = 0; i < size_in_words; i++) {
        if (jtag_transfer_retry(req, (uint32_t *)data) != 0x01) {
            return 0;
        }

        data += 4;
    }

    // dummy read

    return 1;
}

// Read 32-bit word aligned values from target memory using address auto-increment.
// size is in bytes.
static uint8_t jtag_read_block(uint32_t address, uint8_t *data, uint32_t size)
{
    uint8_t tmp_in[4], req, ack;
    uint32_t size_in_words;
    uint32_t i;

    if (size == 0) {
        return 0;
    }

    jtag_write_ir(JTAG_APACC);

    size_in_words = size / 4;

    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32)) {
        return 0;
    }

    // TAR write
    req = JTAG_REG_ADR(AP_TAR);
    __int2array(tmp_in, address, 4);

    if (jtag_transfer_retry(req, (uint32_t *)tmp_in) != DAP_TRANSFER_OK) {
        return 0;
    }

    // read data
    req = DAP_TRANSFER_RnW | JTAG_REG_ADR(AP_DRW);

    // initiate first read, data comes back in next read
    if (jtag_transfer_retry(req, NULL) != 0x01) {
        return 0;
    }

    for (i = 0; i < (size_in_words - 1); i++) {
        if (jtag_transfer_retry(req, (uint32_t *)data) != DAP_TRANSFER_OK) {
            return 0;
        }

        data += 4;
    }

    // read last word
    ack = jtag_transfer_retry(req, (uint32_t *)data);

    return (ack == 0x01);
}

// Read target memory.
static uint8_t jtag_read_data(uint32_t addr, uint32_t *val)
{
    uint8_t tmp_in[4];
    uint8_t tmp_out[4];
    uint8_t req, ack;
    uint32_t tmp;

    jtag_write_ir(JTAG_APACC);

    // put addr in TAR register
    __int2array(tmp_in, addr, 4);

    if (jtag_transfer_retry(JTAG_REG_ADR(AP_TAR), (uint32_t *)tmp_in) != 0x01) {
        return 0;
    }

    req = DAP_TRANSFER_RnW | JTAG_REG_ADR(AP_DRW);
    // first dummy read
    jtag_transfer_retry(req, (uint32_t *)tmp_out);
    // read data from DRW register
    ack = jtag_transfer_retry(req, (uint32_t *)tmp_out);
    *val = 0;
    tmp = tmp_out[3];
    *val |= (tmp << 24);
    tmp = tmp_out[2];
    *val |= (tmp << 16);
    tmp = tmp_out[1];
    *val |= (tmp << 8);
    tmp = tmp_out[0];
    *val |= (tmp << 0);
    return (ack == 0x01);
}

// Write target memory.
static uint8_t jtag_write_data(uint32_t address, uint32_t data)
{
    uint8_t tmp_in[4];

    jtag_write_ir(JTAG_APACC);

    // put addr in TAR register
    __int2array(tmp_in, address, 4);

    if (jtag_transfer_retry(JTAG_REG_ADR(AP_TAR), (uint32_t *)tmp_in) != 0x01) {
        return 0;
    }

    // write data
    __int2array(tmp_in, data, 4);

    if (jtag_transfer_retry(JTAG_REG_ADR(AP_DRW), (uint32_t *)tmp_in) != 0x01) {
        return 0;
    }

    return 1;
}

// Read 32-bit word from target memory.
uint8_t jtag_read_word(uint32_t addr, uint32_t *val)
{
    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32)) {
        return 0;
    }

    if (!jtag_read_data(addr, val)) {
        return 0;
    }

    return 1;
}

// Write 32-bit word to target memory.
uint8_t jtag_write_word(uint32_t addr, uint32_t val)
{
    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32)) {
        return 0;
    }

    if (!jtag_write_data(addr, val)) {
        return 0;
    }

    return 1;
}

// Read 8-bit byte from target memory.
uint8_t jtag_read_byte(uint32_t addr, uint8_t *val)
{
    uint32_t tmp;

    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE8)) {
        return 0;
    }

    if (!jtag_read_data(addr, &tmp)) {
        return 0;
    }

    *val = (uint8_t)(tmp >> ((addr & 0x03) << 3));
    return 1;
}

// Write 8-bit byte to target memory.
uint8_t jtag_write_byte(uint32_t addr, uint8_t val)
{
    uint32_t tmp;

    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE8)) {
        return 0;
    }

    tmp = val << ((addr & 0x03) << 3);

    if (!jtag_write_data(addr, tmp)) {
        return 0;
    }

    return 1;
}

// Read unaligned data from target memory.
// size is in bytes.
uint8_t jtag_read_memory(uint32_t address, uint8_t *data, uint32_t size)
{
    uint32_t n;

    // Read bytes until word aligned
    while ((size > 0) && (address & 0x3)) {
        if (!jtag_read_byte(address, data)) {
            return 0;
        }

        address++;
        data++;
        size--;
    }

    // Read word aligned blocks
    while (size > 3) {
        // Limit to auto increment page size
        n = TARGET_AUTO_INCREMENT_PAGE_SIZE - (address & (TARGET_AUTO_INCREMENT_PAGE_SIZE - 1));

        if (size < n) {
            n = size & 0xFFFFFFFC; // Only count complete words remaining
        }

        if (!jtag_read_block(address, data, n)) {
            return 0;
        }

        address += n;
        data += n;
        size -= n;
    }

    // Read remaining bytes
    while (size > 0) {
        if (!jtag_read_byte(address, data)) {
            return 0;
        }

        address++;
        data++;
        size--;
    }

    return 1;
}

// Write unaligned data to target memory.
// size is in bytes.
uint8_t jtag_write_memory(uint32_t address, uint8_t *data, uint32_t size)
{
    uint32_t n = 0;

    // Write bytes until word aligned
    while ((size > 0) && (address & 0x3)) {
        if (!jtag_write_byte(address, *data)) {
            return 0;
        }

        address++;
        data++;
        size--;
    }

    // Write word aligned blocks
    while (size > 3) {
        // Limit to auto increment page size
        n = TARGET_AUTO_INCREMENT_PAGE_SIZE - (address & (TARGET_AUTO_INCREMENT_PAGE_SIZE - 1));

        if (size < n) {
            n = size & 0xFFFFFFFC; // Only count complete words remaining
        }

        if (!jtag_write_block(address, data, n)) {
            return 0;
        }

        address += n;
        data += n;
        size -= n;
    }

    // Write remaining bytes
    while (size > 0) {
        if (!jtag_write_byte(address, *data)) {
            return 0;
        }

        address++;
        data++;
        size--;
    }

    return 1;
}

uint8_t jtag_read_core_register(uint32_t n, uint32_t *val)
{
    int i = 0, timeout = 100;

    if (!jtag_write_word(DCRSR, n)) {
        return 0;
    }

    // wait for S_REGRDY
    for (i = 0; i < timeout; i++) {
        if (!jtag_read_word(DHCSR, val)) {
            return 0;
        }

        if (*val & S_REGRDY) {
            break;
        }
    }

    if (i == timeout) {
        return 0;
    }

    if (!jtag_read_word(DCRDR, val)) {
        return 0;
    }

    return 1;
}

uint8_t jtag_write_core_register(uint32_t n, uint32_t val)
{
    int i = 0, timeout = 100;

    if (!jtag_write_word(DCRDR, val)) {
        return 0;
    }

    if (!jtag_write_word(DCRSR, n | REGWnR)) {
        return 0;
    }

    // wait for S_REGRDY
    for (i = 0; i < timeout; i++) {
        if (!jtag_read_word(DHCSR, &val)) {
            return 0;
        }

        if (val & S_REGRDY) {
            return 1;
        }
    }

    return 0;
}

// Execute system call.
static uint8_t jtag_write_debug_state(DEBUG_STATE *state)
{
    uint32_t i, status;

    if (!jtag_write_dp(DP_SELECT, 0)) {
        return 0;
    }

    // R0, R1, R2, R3
    for (i = 0; i < 4; i++) {
        if (!jtag_write_core_register(i, state->r[i])) {
            return 0;
        }
    }

    // R9
    if (!jtag_write_core_register(9, state->r[9])) {
        return 0;
    }

    // R13, R14, R15
    for (i = 13; i < 16; i++) {
        if (!jtag_write_core_register(i, state->r[i])) {
            return 0;
        }
    }

    // xPSR
    if (!jtag_write_core_register(16, state->xpsr)) {
        return 0;
    }

    if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_MASKINTS | C_HALT)) {
        return 0;
    }

    if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_MASKINTS)) {
        return 0;
    }

    // check status
    if (!jtag_read_dp(DP_CTRL_STAT, &status)) {
        return 0;
    }

    if (status & (STICKYERR | WDATAERR)) {
        return 0;
    }

    return 1;
}


static uint8_t jtag_wait_until_halted(void)
{
    // Wait for target to stop
    uint32_t val, i, timeout = MAX_TIMEOUT;

    for (i = 0; i < timeout; i++) {
        if (!jtag_read_word(DBG_HCSR, &val)) {
            return 0;
        }

        if (val & S_HALT) {
            return 1;
        }
    }

    return 0;
}

uint8_t jtag_flash_syscall_exec(const program_syscall_t *sysCallParam, uint32_t entry, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4, jtag_flash_algo_return_t return_type)
{
    DEBUG_STATE state = {{0}, 0};
    // Call flash algorithm function on target and wait for result.
    state.r[0]     = arg1;                   // R0: Argument 1
    state.r[1]     = arg2;                   // R1: Argument 2
    state.r[2]     = arg3;                   // R2: Argument 3
    state.r[3]     = arg4;                   // R3: Argument 4
    state.r[9]     = sysCallParam->static_base;    // SB: Static Base
    state.r[13]    = sysCallParam->stack_pointer;  // SP: Stack Pointer
    state.r[14]    = sysCallParam->breakpoint;     // LR: Exit Point
    state.r[15]    = entry;                        // PC: Entry Point
    state.xpsr     = 0x01000000;          // xPSR: T = 1, ISR = 0

    if (!jtag_write_debug_state(&state)) {
        return 0;
    }

    if (!jtag_wait_until_halted()) {
        return 0;
    }

    if (!jtag_read_core_register(0, &state.r[0])) {
        return 0;
    }

    //remove the C_MASKINTS
    if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT)) {
        return 0;
    }

    if ( return_type == JTAG_FLASHALGO_RETURN_POINTER ) {
        // Flash verify functions return pointer to byte following the buffer if successful.
        if (state.r[0] != (arg1 + arg2)) {
            return 0;
        }
    }
    else {
        // Flash functions return 0 if successful.
        if (state.r[0] != 0) {
            return 0;
        }
    }

    return 1;
}

// JTAG Reset
static uint8_t jtag_reset(void)
{
    uint8_t tmp_in[8];
    uint8_t i = 0;

    for (i = 0; i < 8; i++) {
        tmp_in[i] = 0xff;
    }

    SWJ_Sequence(51 , tmp_in);
    return 1;
}

// JTAG Switch
static uint8_t jtag_switch(uint16_t val)
{
    uint8_t tmp_in[2];
    tmp_in[0] = val & 0xff;
    tmp_in[1] = (val >> 8) & 0xff;
    SWJ_Sequence(16, tmp_in);
    return 1;
}

// JTAG Read ID
static uint8_t jtag_read_idcode(uint32_t *id)
{
    uint8_t tmp_in[1];
    tmp_in[0] = 0x00;

    SWJ_Sequence(8, tmp_in);

    jtag_write_ir(JTAG_IDCODE);

    *id = JTAG_ReadIDCode();

    return 1;
}


uint8_t SWD2JTAG(void)
{
    uint32_t tmp = 0;

    if (!jtag_reset()) {
        return 0;
    }

    if (!jtag_switch(0xE73C)) {
        return 0;
    }

    if (!jtag_reset()) {
        return 0;
    }

    if (!jtag_read_idcode(&tmp)) {
        return 0;
    }

    return 1;
}

uint8_t jtag_init_debug(void)
{
    uint32_t tmp = 0;
    int i = 0;
    int timeout = 100;
    // init dap state with fake values
    dap_state.select = 0xffffffff;
    dap_state.csw = 0xffffffff;

    int8_t retries = 4;
    int8_t do_abort = 0;
    do {
        if (do_abort) {
            //do an abort on stale target, then reset the device
            jtag_write_dp(DP_ABORT, DAPABORT);
            swd_set_target_reset(1);
            osDelay(2);
            swd_set_target_reset(0);
            osDelay(2);
            do_abort = 0;
        }
        jtag_init();

        // call a target dependant function
        // this function can do several stuff before really
        // initing the debug
        if (g_target_family && g_target_family->target_before_init_debug) {
            g_target_family->target_before_init_debug();
        }

        if (!SWD2JTAG()) {
            do_abort = 1;
            continue;
        }

        if (!jtag_clear_errors()) {
            do_abort = 1;
            continue;
        }

        if (!jtag_write_dp(DP_SELECT, 0)) {
            do_abort = 1;
            continue;

        }

        // Power up
        if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)) {
            do_abort = 1;
            continue;
        }

        for (i = 0; i < timeout; i++) {
            if (!jtag_read_dp(DP_CTRL_STAT, &tmp)) {
                do_abort = 1;
                break;
            }
            if ((tmp & (CDBGPWRUPACK | CSYSPWRUPACK)) == (CDBGPWRUPACK | CSYSPWRUPACK)) {
                // Break from loop if powerup is complete
                break;
            }
        }
        if ((i == timeout) || (do_abort == 1)) {
            // Unable to powerup DP
            do_abort = 1;
            continue;
        }

        if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ | TRNNORMAL | MASKLANE)) {
            do_abort = 1;
            continue;
        }

        // call a target dependant function:
        // some target can enter in a lock state
        // this function can unlock these targets
        if (g_target_family && g_target_family->target_unlock_sequence) {
            g_target_family->target_unlock_sequence();
        }

        if (!jtag_write_dp(DP_SELECT, 0)) {
            do_abort = 1;
            continue;
        }

        return 1;

    } while (--retries > 0);

    return 0;
}

uint8_t jtag_set_target_state_hw(target_state_t state)
{
    uint32_t val;
    int8_t ap_retries = 2;
    /* Calling jtag_init prior to entering RUN state causes operations to fail. */
    if (state != RUN) {
        jtag_init();
    }

    switch (state) {
        case RESET_HOLD:
            swd_set_target_reset(1);
            break;

        case RESET_RUN:
            swd_set_target_reset(1);
            osDelay(2);
            swd_set_target_reset(0);
            osDelay(2);
            jtag_off();
            break;

        case RESET_PROGRAM:
            if (!jtag_init_debug()) {
                return 0;
            }

            // Enable debug
            while(jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN) == 0) {
                if( --ap_retries <=0 )
                    return 0;
                // Target is in invalid state?
                swd_set_target_reset(1);
                osDelay(2);
                swd_set_target_reset(0);
                osDelay(2);
            }

            // Enable halt on reset
            if (!jtag_write_word(DBG_EMCR, VC_CORERESET)) {
                return 0;
            }

            // Deassert reset
            swd_set_target_reset(0);
            osDelay(2);

            do {
                if (!jtag_read_word(DBG_HCSR, &val)) {
                    return 0;
                }
            } while ((val & S_HALT) == 0);

            // Disable halt on reset
            if (!jtag_write_word(DBG_EMCR, 0)) {
                return 0;
            }

            break;

        case NO_DEBUG:
            if (!jtag_write_word(DBG_HCSR, DBGKEY)) {
                return 0;
            }

            break;

        case DEBUG:
            if (!SWD2JTAG()) {
                return 0;
            }

            if (!jtag_clear_errors()) {
                return 0;
            }

            // Ensure CTRL/STAT register selected in DPBANKSEL
            if (!jtag_write_dp(DP_SELECT, 0)) {
                return 0;
            }

            // Power up
            if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)) {
                return 0;
            }

            // Enable debug
            if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN)) {
                return 0;
            }

            break;

        case HALT:
            if (!jtag_init_debug()) {
                return 0;
            }

            // Enable debug and halt the core (DHCSR <- 0xA05F0003)
            if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT)) {
                return 0;
            }

            // Wait until core is halted
            do {
                if (!jtag_read_word(DBG_HCSR, &val)) {
                    return 0;
                }
            } while ((val & S_HALT) == 0);
            break;

        case RUN:
            if (!jtag_write_word(DBG_HCSR, DBGKEY)) {
                return 0;
            }
            jtag_off();
            break;

        case POST_FLASH_RESET:
            // This state should be handled in target_reset.c, nothing needs to be done here.
            break;

        default:
            return 0;
    }

    return 1;
}

uint8_t jtag_set_target_state_sw(target_state_t state)
{
    uint32_t val;
    int8_t ap_retries = 2;
    /* Calling jtag_init prior to enterring RUN state causes operations to fail. */
    if (state != RUN) {
        jtag_init();
    }

    switch (state) {
        case RESET_HOLD:
            swd_set_target_reset(1);
            break;

        case RESET_RUN:
            swd_set_target_reset(1);
            osDelay(2);
            swd_set_target_reset(0);
            osDelay(2);

            if (!jtag_init_debug()) {
                return 0;
            }

            // Power down
            // Per ADIv6 spec. Clear first CSYSPWRUPREQ followed by CDBGPWRUPREQ
            if (!jtag_read_dp(DP_CTRL_STAT, &val)) {
                return 0;
            }

            if (!jtag_write_dp(DP_CTRL_STAT, val & ~CSYSPWRUPREQ)) {
                return 0;
            }

            // Wait until ACK is deasserted
            do {
                if (!jtag_read_dp(DP_CTRL_STAT, &val)) {
                    return 0;
                }
            } while ((val & (CSYSPWRUPACK)) != 0);

            if (!jtag_write_dp(DP_CTRL_STAT, val & ~CDBGPWRUPREQ)) {
                return 0;
            }

            // Wait until ACK is deasserted
            do {
                if (!jtag_read_dp(DP_CTRL_STAT, &val)) {
                    return 0;
                }
            } while ((val & (CDBGPWRUPACK)) != 0);

            jtag_off();
            break;

        case RESET_PROGRAM:
            if (!jtag_init_debug()) {
                return 0;
            }

            // Enable debug and halt the core (DHCSR <- 0xA05F0003)
            while (jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT) == 0) {
                if ( --ap_retries <=0 ) {
                    return 0;
                }
                // Target is in invalid state?
                swd_set_target_reset(1);
                osDelay(2);
                swd_set_target_reset(0);
                osDelay(2);
            }

            // Wait until core is halted
            do {
                if (!jtag_read_word(DBG_HCSR, &val)) {
                    return 0;
                }
            } while ((val & S_HALT) == 0);

            // Enable halt on reset
            if (!jtag_write_word(DBG_EMCR, VC_CORERESET)) {
                return 0;
            }

            // Perform a soft reset
            if (!jtag_read_word(NVIC_AIRCR, &val)) {
                return 0;
            }

            if (!jtag_write_word(NVIC_AIRCR, VECTKEY | (val & SCB_AIRCR_PRIGROUP_Msk) | soft_reset)) {
                return 0;
            }

            osDelay(2);

            do {
                if (!jtag_read_word(DBG_HCSR, &val)) {
                    return 0;
                }
            } while ((val & S_HALT) == 0);

            // Disable halt on reset
            if (!jtag_write_word(DBG_EMCR, 0)) {
                return 0;
            }

            break;

        case NO_DEBUG:
            if (!jtag_write_word(DBG_HCSR, DBGKEY)) {
                return 0;
            }

            break;

        case DEBUG:
            if (!SWD2JTAG()) {
                return 0;
            }

            if (!jtag_clear_errors()) {
                return 0;
            }

            // Ensure CTRL/STAT register selected in DPBANKSEL
            if (!jtag_write_dp(DP_SELECT, 0)) {
                return 0;
            }

            // Power up
            if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)) {
                return 0;
            }

            // Enable debug
            if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN)) {
                return 0;
            }

            break;

        case HALT:
            if (!jtag_init_debug()) {
                return 0;
            }

            // Enable debug and halt the core (DHCSR <- 0xA05F0003)
            if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT)) {
                return 0;
            }

            // Wait until core is halted
            do {
                if (!jtag_read_word(DBG_HCSR, &val)) {
                    return 0;
                }
            } while ((val & S_HALT) == 0);
            break;

        case RUN:
            if (!jtag_write_word(DBG_HCSR, DBGKEY)) {
                return 0;
            }
            jtag_off();
            break;

        case POST_FLASH_RESET:
            // This state should be handled in target_reset.c, nothing needs to be done here.
            break;

        default:
            return 0;
    }

    return 1;
}
#endif
