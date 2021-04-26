//*****************************************************************************
//
//! @file am_util_ble_cooper.c
//!
//! @brief Useful BLE functions not covered by the HAL.
//!
//! This file contains functions for interacting with the Apollo4 BLE hardware
//! that are not already covered by the HAL. Most of these commands either
//! adjust RF settings or facilitate RF testing operations.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2021, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision b0-release-20210111-833-gc25608de46 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_util_delay.h"
#include "am_mcu_apollo.h"
#include "am_devices_cooper.h"

//*****************************************************************************
//
// Statics
//
//*****************************************************************************
//*****************************************************************************
//
// Builds a vendor-specific BLE command.
//
//*****************************************************************************
uint32_t
am_devices_cooper_command_write(void* pHandle, uint32_t* pui32Command, uint32_t* pui32Response, uint32_t ui32OpCode,
                                uint32_t ui32TotalLength, uint8_t* pui8Parameters)
{
    uint8_t* pui8Dest = (uint8_t*) pui32Command;
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    uint32_t ui32NumChars;
    //
    // Build the header portion of the command from the given argments.
    //
    pui8Dest[0] = 0x01;
    pui8Dest[1] = ui32OpCode & 0xFF;
    pui8Dest[2] = (ui32OpCode >> 8) & 0xFF;
    pui8Dest[3] = (ui32TotalLength - 4) & 0xFF;
    //
    // Finish filling the array with any parameters that may be required.
    //
    if ( pui8Parameters )
    {
        memcpy(&pui8Dest[4], pui8Parameters, ui32TotalLength - 4);
    }
    ui32ErrorStatus = am_devices_cooper_blocking_write(pHandle,
                      AM_DEVICES_COOPER_RAW,
                      pui32Command,
                      ui32TotalLength, true);

    if (ui32ErrorStatus)
    {
        return AM_DEVICES_COOPER_STATUS_PACKET_INCOMPLETE;
    }
    //
    // Wait for the response, and return it to the caller via our variable.
    //
    WHILE_TIMEOUT_MS ( am_devices_cooper_irq_read() == 0, 50, ui32ErrorStatus );
    if ( ui32ErrorStatus )
    {
        return AM_DEVICES_COOPER_STATUS_NO_RESPONSE;
    }
    ui32ErrorStatus = am_devices_cooper_blocking_read(pHandle, pui32Response, &ui32NumChars);

    if (ui32ErrorStatus)
    {
        return AM_DEVICES_COOPER_STATUS_PACKET_INCOMPLETE;
    }
    //
    // Return the status.
    //
    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Read a register value from the BLE core.
//
//*****************************************************************************
uint32_t
am_devices_cooper_plf_reg_read(void* pHandle, uint32_t ui32Address, uint32_t* pui32Value)
{
    uint8_t pui8Parameter[4];
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    am_devices_cooper_buffer(AM_DEVICES_COOPER_PLF_REGISTER_READ_LENGTH) sWriteCommand;
    am_devices_cooper_buffer(32) sResponse;
    //
    // Prepare our register write value.
    //
    pui8Parameter[0] = ui32Address;
    pui8Parameter[1] = (ui32Address >> 8);
    pui8Parameter[2] = (ui32Address >> 16);
    pui8Parameter[3] = (ui32Address >> 24);
    //pui8Parameter[4] = 0x20; //RD_32_Bit, 1 word
    //pui8Parameter[5] = 1;
    sResponse.words[0] = 0;
    sResponse.words[1] = 0;
    sResponse.words[2] = 0;
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    ui32ErrorStatus = am_devices_cooper_command_write(pHandle, sWriteCommand.words, sResponse.words,
                                       AM_DEVICES_COOPER_PLF_REGISTER_READ_OPCODE,
                                       AM_DEVICES_COOPER_PLF_REGISTER_READ_LENGTH,
                                       pui8Parameter);
    *pui32Value = (((sResponse.words[2] & 0xFF000000) >> 24) |
                   ((sResponse.words[3] & 0x00FFFFFF) << 8));
    return ui32ErrorStatus;
}

//*****************************************************************************
//
// Write a register value to the BLE core.
//
//*****************************************************************************
uint32_t
am_devices_cooper_plf_reg_write(void* pHandle, uint32_t ui32Address, uint32_t ui32Value)
{
    uint8_t pui8Parameter[10];
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    am_devices_cooper_buffer(AM_DEVICES_COOPER_PLF_REGISTER_WRITE_LENGTH) sWriteCommand;
    am_devices_cooper_buffer(16) sResponse;
    //
    // Prepare our register write value.
    //
    pui8Parameter[0] = ui32Address;
    pui8Parameter[1] = (ui32Address >> 8);
    pui8Parameter[2] = (ui32Address >> 16);
    pui8Parameter[3] = (ui32Address >> 24);
    pui8Parameter[4] = ui32Value;
    pui8Parameter[5] = (ui32Value >> 8);
    pui8Parameter[6] = (ui32Value >> 16);
    pui8Parameter[7] = (ui32Value >> 24);
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    ui32ErrorStatus = am_devices_cooper_command_write(pHandle, sWriteCommand.words, sResponse.words,
                                       AM_DEVICES_COOPER_PLF_REGISTER_WRITE_OPCODE,
                                       AM_DEVICES_COOPER_PLF_REGISTER_WRITE_LENGTH,
                                       pui8Parameter);
    return ui32ErrorStatus;
}

//*****************************************************************************
//
// Set BLE sleep enable/disable for the BLE core.
// enable = 'true' set sleep enable, enable = 'false' set sleep disable
//
//*****************************************************************************
uint32_t
am_devices_cooper_sleep_set(void* pHandle, bool enable)
{
    uint8_t pui8Parameter[] = AM_DEVICES_COOPER_UPDATE_NVDS_PARA(0x11, 0x06, 0x01, enable);
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    am_devices_cooper_buffer(AM_DEVICES_COOPER_UPDATE_NVDS_LENGTH) sWriteCommand;
    am_devices_cooper_buffer(16) sResponse;
    memset(sWriteCommand.bytes, 0, AM_DEVICES_COOPER_UPDATE_NVDS_LENGTH);
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    ui32ErrorStatus = am_devices_cooper_command_write(pHandle, sWriteCommand.words, sResponse.words,
                                       AM_DEVICES_COOPER_UPDATE_NVDS_OPCODE,
                                       AM_DEVICES_COOPER_UPDATE_NVDS_LENGTH,
                                       pui8Parameter);
    return ui32ErrorStatus;
}

//*****************************************************************************
//
// set the tx power of BLE
// values.
// ui32TxPower: 0x03->-20dBm 0x04->-10dBm 0x05->-5dBm 0x08->0dBm 0x0F->4dBm
//
//*****************************************************************************
uint32_t
am_devices_cooper_tx_power_set(void* pHandle, uint8_t ui32TxPower)
{
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    am_devices_cooper_buffer(AM_DEVICES_COOPER_SET_TX_POWER_LENGTH) sWriteCommand;
    am_devices_cooper_buffer(16) sResponse;
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    ui32ErrorStatus = am_devices_cooper_command_write(pHandle, sWriteCommand.words, sResponse.words,
                                       AM_DEVICES_COOPER_SET_TX_POWER_OPCODE,
                                       AM_DEVICES_COOPER_SET_TX_POWER_LENGTH,
                                       &ui32TxPower);
    return ui32ErrorStatus;
}


