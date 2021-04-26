//*****************************************************************************
//
//! @file am_devices_cooper.c
//!
//! @brief A implementation of the Apollo inteface to Cooper using the IOM.
//!
//! @addtogroup
//! @ingroup
//! @{
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_devices_cooper.h"

//*****************************************************************************
//
// Timing and configuration.
//
//*****************************************************************************
#define AM_DEVICES_COOPER_TIMEOUT                 10
#define AM_DEVICES_COOPER_RETRY_TIMES     0xFFFFFFFF

#define OPCODE_H2WRITE_HANDSHAKE   0x80
#define OPCODE_H2READ_HANDSHAKE    0x04
#define OPCODE_READ_STATUS         0x08

// Set to 1 to turn on logging for SBL diagnosis
#define SBL_DEBUG_LOG_ON           0

//*****************************************************************************
//
// Global state variables.
//
//*****************************************************************************
am_devices_cooper_t gAmCooper[AM_DEVICES_COOPER_MAX_DEVICE_NUM];

//*****************************************************************************
//
// Global SBL update data.
//
//*****************************************************************************
am_devices_cooper_buffer(2) sLengthBytes;

static am_devices_cooper_sbl_update_state_t gsSblUpdateState;

static am_devices_cooper_sbl_update_data_t     g_sFwImage =
{
    0,
    0,
    AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW,
    AM_DEVICES_COOPER_SBL_FW_VERSION

};

static am_devices_cooper_sbl_update_data_t     g_sInfo0PatchImage =
{
    0,
    sizeof(am_sbl_info0_patch_blob_t),
    AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0,
    AM_DEVICES_COOPER_SBL_FW_VERSION
};

static am_devices_cooper_sbl_update_data_t     g_sInfo1PatchImage =
{
    0,
    0,
    AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1,
    AM_DEVICES_COOPER_SBL_FW_VERSION
};

#if (!AM_DEVICES_COOPER_QFN_PART)
//*****************************************************************************
//
// BLE_32M_CLK (46) - BLE 32M CLK OUT.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_DEVICES_COOPER_32M_CLK =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_46_CLKOUT_32M,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//*****************************************************************************
//
// BLE_32K_CLK (45) - BLE 32K CLK OUT.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_DEVICES_COOPER_32K_CLK =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_45_32KHzXT,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};
#endif

//*****************************************************************************
//
// BLE_CLKREQ (40) - BLE CLK request pin.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_DEVICES_COOPER_CLKREQ =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_40_GPIO,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//*****************************************************************************
//
// Mark the BLE interface busy so it doesn't get used by more than one
// interface.
//
//*****************************************************************************
static bool
am_devices_cooper_bus_lock(void* pHandle)
{
    bool bLockObtained;
    am_devices_cooper_t* pBle = (am_devices_cooper_t*)pHandle;
    if (pBle->bBusy == false)
    {
        pBle->bBusy = true;
        bLockObtained = true;
    }
    else
    {
        bLockObtained = false;
    }
    //
    // Tell the caller if we successfully locked the bus.
    //
    return bLockObtained;
}

//*****************************************************************************
//
// Release the bus so someone else can use it.
//
//*****************************************************************************
static void
am_devices_cooper_bus_release(void* pHandle)
{
    am_devices_cooper_t* pBle = (am_devices_cooper_t*)pHandle;
    pBle->bBusy = false;
}

static uint32_t sbl_status = 0;

//*****************************************************************************
//
//! @brief Set up pins for Cooper.
//!
//! This function configures SPI, IRQ, SCK pins for Cooper
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_cooper_pins_enable(void)
{
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_RESET_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_IRQ_PIN, am_hal_gpio_pincfg_input);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_CLKREQ_PIN, g_AM_DEVICES_COOPER_CLKREQ);
#if (!AM_DEVICES_COOPER_QFN_PART)
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SWDIO, am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SWCLK, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_SWDIO);
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_SWCLK);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32M_CLK, g_AM_DEVICES_COOPER_32M_CLK);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32K_CLK, g_AM_DEVICES_COOPER_32K_CLK);
#else
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32M_OSCEN_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_set(AM_DEVICES_COOPER_32M_OSCEN_PIN);
    am_util_stdio_printf("\nThe Cooper QFN board is attached for debug\n");
#endif
}

//*****************************************************************************
//
//! @brief Disable pins for Cooper.
//!
//! This function configures SPI, IRQ, SCK pins for Cooper
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_cooper_pins_disable(void)
{
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_RESET_PIN);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_RESET_PIN, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_IRQ_PIN, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_CLKREQ_PIN, am_hal_gpio_pincfg_disabled);
#if (!AM_DEVICES_COOPER_QFN_PART)
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_SWDIO);
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_SWCLK);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SWDIO, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SWCLK, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32M_CLK, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32K_CLK, am_hal_gpio_pincfg_disabled);
#else
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_32M_OSCEN_PIN);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32M_OSCEN_PIN, am_hal_gpio_pincfg_disabled);
#endif
}

//*****************************************************************************
//
//! @brief Initialize the BLE controller driver.
//!
//! @param ui32Module     - IOM Module#
//! @param psIOMSettings  - IOM device structure describing the target.
//!
//! This function should be called before any other am_devices_cooper
//! functions. It is used to set tell the other functions how to communicate
//! with the BLE controller hardware.
//!
//! @return Status.
//
//*****************************************************************************
uint32_t
am_devices_cooper_init(uint32_t ui32Module, am_devices_cooper_config_t* pDevConfig, void** ppHandle, void** ppBleHandle)
{
    void* pBleHandle;
    am_hal_iom_config_t     stIOMCOOPERSettings;
    uint32_t      ui32Index = 0;
    uint32_t g_CS[AM_REG_IOM_NUM_MODULES] =
    {
        0, 0, 0, 0, 0, 0, 0, 0
    };
    // Allocate a vacant device handle
    for ( ui32Index = 0; ui32Index < AM_DEVICES_COOPER_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAmCooper[ui32Index].bOccupied == false )
        {
            break;
        }
    }
    if ( ui32Index == AM_DEVICES_COOPER_MAX_DEVICE_NUM )
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }
    if ( (ui32Module > AM_REG_IOM_NUM_MODULES)  || (pDevConfig == NULL) )
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }
    //
    // Enable fault detection.
    //
    am_hal_fault_capture_enable();
    stIOMCOOPERSettings.ui32ClockFreq        = COOPER_IOM_FREQ;
    stIOMCOOPERSettings.eInterfaceMode       = AM_HAL_IOM_SPI_MODE,
    stIOMCOOPERSettings.eSpiMode             = AM_HAL_IOM_SPI_MODE_3,
    stIOMCOOPERSettings.ui32NBTxnBufLength   = pDevConfig->ui32NBTxnBufLength;
    stIOMCOOPERSettings.pNBTxnBuf            = pDevConfig->pNBTxnBuf;
    //
    // Initialize the IOM instance.
    // Enable power to the IOM instance.
    // Configure the IOM for Serial operation during initialization.
    // Enable the IOM.
    // HAL Success return is 0
    //
    if (am_hal_iom_initialize(ui32Module, &pBleHandle) ||
            am_hal_iom_power_ctrl(pBleHandle, AM_HAL_SYSCTRL_WAKE, false) ||
            am_hal_iom_configure(pBleHandle, &stIOMCOOPERSettings) ||
            am_hal_iom_enable(pBleHandle))
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }
    else
    {
        //
        // Configure the IOM pins.
        //
#if (AM_DEVICES_COOPER_QFN_PART)
#if defined(AM_BSP_GPIO_IOM2_CS)
#undef AM_BSP_GPIO_IOM2_CS
#define AM_BSP_GPIO_IOM2_CS  AM_DEVICES_COOPER_SPI_CS // BGA&SIP share the same CS pin(NCE72) on the QFN shiled board
#endif
#endif
        am_bsp_iom_pins_enable(ui32Module, AM_HAL_IOM_SPI_MODE);
        am_devices_cooper_pins_enable();
#if (!AM_DEVICES_COOPER_QFN_PART)
        // Enable crystals for Cooper
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_ENABLE, 0);

        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, 0);
#endif
        am_devices_cooper_reset();

        gAmCooper[ui32Index].pfnCallback = NULL;
        gAmCooper[ui32Index].pCallbackCtxt = NULL;
        gAmCooper[ui32Index].bBusy = false;
        gAmCooper[ui32Index].bOccupied = true;
        gAmCooper[ui32Index].bNeedCallback = true;
        gAmCooper[ui32Index].bDMAComplete = false;
        gAmCooper[ui32Index].bWakingUp = false;
        gAmCooper[ui32Index].ui32CS = g_CS[ui32Module];
        gAmCooper[ui32Index].ui32Module = ui32Module;
        *ppBleHandle = gAmCooper[ui32Index].pBleHandle = pBleHandle;
        *ppHandle = (void*)&gAmCooper[ui32Index];
        // SBL checking
        am_devices_cooper_image_update_init(*ppHandle, pDevConfig->pNBTxnBuf);
        sbl_status = AM_DEVICES_COOPER_SBL_STATUS_INIT;
        sbl_status = am_devices_cooper_update_image();

        while( (sbl_status != AM_DEVICES_COOPER_SBL_STATUS_OK) &&
                ( sbl_status != AM_DEVICES_COOPER_SBL_STATUS_FAIL) )
        {
            while (am_devices_cooper_irq_read() == 0)
            {
                am_hal_delay_us(50);
            }
            sbl_status = am_devices_cooper_update_image();
        }
        //
        // Return the status.
        //
        if (sbl_status == AM_DEVICES_COOPER_SBL_STATUS_OK)
        {
            // need to wait a bit to jump from SBL to Cooper application firmware
            am_util_delay_ms(10);
            am_util_stdio_printf("SBL Done\r\n");
            return AM_DEVICES_COOPER_STATUS_SUCCESS;
        }
        else
        {
            // free up resource that won't be used.
            am_devices_cooper_term(*ppHandle);
            *ppHandle = NULL;
            am_util_stdio_printf("SBL Cooper Error 0x%x\r\n", sbl_status);
            return gsSblUpdateState.ui32CooperSblStatus;
        }
    }
}

//*****************************************************************************
//
//! @brief De-Initialize the BLE controller driver.
//!
//! @param ui32Module     - IOM Module#
//!
//! This function reverses the initialization
//!
//! @return Status.
//
//*****************************************************************************
uint32_t
am_devices_cooper_term(void* pHandle)
{
    am_devices_cooper_t* pBle = (am_devices_cooper_t*)pHandle;
    if ( pBle->ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }
#if (!AM_DEVICES_COOPER_QFN_PART)
    // Disable crystals
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_DISABLE, 0);
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_DISABLE, 0);
#endif
    // Disable the pins
    am_bsp_iom_pins_disable(pBle->ui32Module, AM_HAL_IOM_SPI_MODE);
    am_devices_cooper_pins_disable();
    //
    // Disable the IOM.
    //
    am_hal_iom_disable(pBle->pBleHandle);
    //
    // Disable power to and uninitialize the IOM instance.
    //
    // Clear local register values first
    am_hal_iom_power_ctrl(pBle->pBleHandle, AM_HAL_SYSCTRL_WAKE, true);
    am_hal_iom_power_ctrl(pBle->pBleHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    am_hal_iom_uninitialize(pBle->pBleHandle);
    // Free this device handle
    pBle->bOccupied = false;
    // Not in waking up state
    pBle->bWakingUp = false;
    //
    // Return the status.
    //
    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}

void
am_devices_cooper_reset(void)
{
    am_hal_gpio_output_set(AM_DEVICES_COOPER_RESET_PIN);
    am_util_delay_ms(20);
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_RESET_PIN);
    am_util_delay_ms(20);
    am_hal_gpio_output_set(AM_DEVICES_COOPER_RESET_PIN);
    am_util_delay_ms(700);
}

uint32_t
am_devices_cooper_awake(void* pHandle)
{
    am_devices_cooper_t* pBle = (am_devices_cooper_t*)pHandle;
    if ( pBle->bOccupied != true )
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }
    am_hal_iom_power_ctrl(pBle->pBleHandle, AM_HAL_SYSCTRL_WAKE, true);
    am_hal_iom_enable(pBle->pBleHandle);
    am_devices_cooper_bus_release(pHandle);
    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}

uint32_t
am_devices_cooper_sleep(void* pHandle)
{
    am_devices_cooper_t* pBle = (am_devices_cooper_t*)pHandle;
    if ( pBle->bOccupied != true )
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }
    //
    // Disable the IOM.
    //
    am_hal_iom_disable(pBle->pBleHandle);
    //
    // Disable power.
    //
    am_hal_iom_power_ctrl(pBle->pBleHandle, AM_HAL_SYSCTRL_DEEPSLEEP, true);
    am_devices_cooper_bus_lock(pHandle);
    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Returns the number of bytes written.
//
//*****************************************************************************
uint32_t
am_devices_cooper_blocking_write(void* pHandle, uint8_t ui8Type, uint32_t* pui32Data,
                                 uint32_t ui32NumBytes, bool bWaitReady)
{
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    memset(&sLengthBytes.bytes, 0, 2);
    //
    // Make a structure for the IOM transfer.
    //
    am_hal_iom_transfer_t sIOMTransfer;
    am_devices_cooper_t *pBle = (am_devices_cooper_t *)pHandle;

    if ( pBle->bWakingUp )
    {
        ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_CONTROLLER_NOT_READY;
        return ui32ErrorStatus;
    }

    am_devices_cooper_awake(pHandle);

    if (!am_devices_cooper_bus_lock(pHandle))
    {
        am_devices_cooper_sleep(pHandle);
        ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_BUS_BUSY;
        return ui32ErrorStatus;
    }
#if defined(AM_PART_APOLLO4B)
    sIOMTransfer.ui64Instr = OPCODE_H2WRITE_HANDSHAKE;
#else
    sIOMTransfer.ui32Instr = OPCODE_H2WRITE_HANDSHAKE;
#endif
    sIOMTransfer.ui32InstrLen = 1;
    sIOMTransfer.eDirection = AM_HAL_IOM_RX;
    sIOMTransfer.ui32NumBytes = 2;
    sIOMTransfer.bContinue = true;
    sIOMTransfer.uPeerInfo.ui32SpiChipSelect = pBle->ui32CS;
    sIOMTransfer.pui32RxBuffer = sLengthBytes.words;
    sIOMTransfer.ui8RepeatCount = 0;
    sIOMTransfer.ui32PauseCondition = 0;
    sIOMTransfer.ui32StatusSetClr = 0;
    do
    {
        if (am_hal_iom_blocking_transfer(pBle->pBleHandle, &sIOMTransfer))
        {
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_PACKET_INCOMPLETE;
            break;
        }
        // Cooper is not ready now
        if ((sLengthBytes.bytes[0] != 0x68) || (sLengthBytes.bytes[1] != 0xA8))
        {
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_CONTROLLER_NOT_READY;
            //
            // Cooper needs CS to low/asserted for 200-300us to detect wakeup request,
            // and it takes about 2ms for Cooper to be ready to accept packet by asserting
            // IRQ pin.
            //
            am_util_delay_us(300);
            // For the applications which do not enable IRQ interrupt, we need to do continuous try here
            if ( bWaitReady )
            {
                //
                // We need to set CS pin (first configured as GPIO) high to trigger Cooper to start wakeup process,
                // after that we reconfigure CS pin back to IOM mode for next transfer.
                //
                am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SPI_CS, am_hal_gpio_pincfg_output);
                am_hal_gpio_output_set(AM_DEVICES_COOPER_SPI_CS);
                am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SPI_CS, g_AM_DEVICES_COOPER_SPI_CS);
                am_util_delay_us(1700);
                continue;
            }
            else
            {
                pBle->bWakingUp = true;
                break;
            }
        }
        //
        // If this isn't a "raw" transaction, we need to make sure the "type" byte
        // gets through to the interface.
        //
        if (ui8Type != AM_DEVICES_COOPER_RAW)
        {
#if defined(AM_PART_APOLLO4B)
            sIOMTransfer.ui64Instr = ui8Type;
#else
            sIOMTransfer.ui32Instr = ui8Type;
#endif
            sIOMTransfer.ui32InstrLen = 1;
        }
        else
        {
#if defined(AM_PART_APOLLO4B)
            sIOMTransfer.ui64Instr = 0;
#else
            sIOMTransfer.ui32Instr = 0;
#endif
            sIOMTransfer.ui32InstrLen = 0;
        }
        sIOMTransfer.eDirection = AM_HAL_IOM_TX;
        sIOMTransfer.ui32NumBytes = ui32NumBytes;
        sIOMTransfer.pui32TxBuffer = pui32Data;
        sIOMTransfer.bContinue = false;
        //
        // If the previous step succeeded, we can go ahead and send the data.
        //
        ui32ErrorStatus = am_hal_iom_blocking_transfer(pBle->pBleHandle, &sIOMTransfer);
        if (ui32ErrorStatus != AM_DEVICES_COOPER_STATUS_SUCCESS)
        {
            //
            // The layer above this one doesn't understand IOM errors, so we
            // will intercept and rename it here.
            //
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_PACKET_INCOMPLETE;
            break;
        }
        break;
    }
    while (1);
    am_devices_cooper_bus_release(pHandle);

    am_devices_cooper_sleep(pHandle);

    return ui32ErrorStatus;
}


//*****************************************************************************
//
// Returns the number of bytes received.
//
//*****************************************************************************
uint32_t
am_devices_cooper_blocking_read(void* pHandle, uint32_t* pui32Data,
                                uint32_t* pui32BytesReceived)
{
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    memset(&sLengthBytes.bytes, 0, 2);
    am_devices_cooper_t* pBle = (am_devices_cooper_t*)pHandle;

    // Skip if IRQ pin is low -- no pending incoimng packet from Cooper.
    if (!am_devices_cooper_irq_read())
    {
        *pui32BytesReceived = 0;
        return AM_DEVICES_COOPER_STATUS_SUCCESS;
    }
    //
    // Make a structure for the IOM transfer.
    //
    am_hal_iom_transfer_t sIOMTransfer;

    am_devices_cooper_awake(pHandle);

    if (!am_devices_cooper_bus_lock(pHandle))
    {
        am_devices_cooper_sleep(pHandle);
        ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_BUS_BUSY;
        return ui32ErrorStatus;
    }
    do
    {
        sIOMTransfer.uPeerInfo.ui32SpiChipSelect = pBle->ui32CS;
#if defined(AM_PART_APOLLO4B)
        sIOMTransfer.ui64Instr = OPCODE_H2READ_HANDSHAKE;
#else
        sIOMTransfer.ui32Instr = OPCODE_H2READ_HANDSHAKE;
#endif
        sIOMTransfer.ui32InstrLen = 1;
        sIOMTransfer.eDirection = AM_HAL_IOM_RX;
        sIOMTransfer.ui32NumBytes = 2;
        sIOMTransfer.pui32RxBuffer = sLengthBytes.words;
        sIOMTransfer.bContinue = true;
        sIOMTransfer.ui8RepeatCount = 0;
        sIOMTransfer.ui32PauseCondition = 0;
        sIOMTransfer.ui32StatusSetClr = 0;
        //
        // First we should get the byte available back
        //
        if (am_hal_iom_blocking_transfer(pBle->pBleHandle, &sIOMTransfer))
        {
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_PACKET_INCOMPLETE;
            break;
        }
        if ((sLengthBytes.bytes[0] == 0) && (sLengthBytes.bytes[1] == 0))
        {
            *pui32BytesReceived = 0;
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
            break;
        }
        //
        // This is the second frame of the read, which contains the actual HCI
        // data.
        //
#if defined(AM_PART_APOLLO4B)
        sIOMTransfer.ui64Instr = 0;
#else
        sIOMTransfer.ui32Instr = 0;
#endif
        sIOMTransfer.ui32InstrLen = 0;
        sIOMTransfer.pui32RxBuffer = pui32Data;
        sIOMTransfer.ui32NumBytes = (sLengthBytes.bytes[0] +
                                     (sLengthBytes.bytes[1] << 8));
        sIOMTransfer.bContinue = false;
        // check ui32NumBytes
        if (sIOMTransfer.ui32NumBytes > AM_DEVICES_COOPER_MAX_RX_PACKET)
        {
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_WRONG_DATA_LENGTH;
            *pui32BytesReceived = 0;
            break;
        }
        //
        // Make sure the caller knows how many bytes we got.
        //
        *pui32BytesReceived = sIOMTransfer.ui32NumBytes;
        //
        // Execute the second part of the transfer.
        //
        ui32ErrorStatus = am_hal_iom_blocking_transfer(pBle->pBleHandle, &sIOMTransfer);
        //
        // A failure here indicates that the second part of the read was bad.
        //
        if (ui32ErrorStatus)
        {
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_PACKET_INCOMPLETE;
            *pui32BytesReceived = 0;
            break;
        }
    }
    while (0);
    am_devices_cooper_bus_release(pHandle);

    am_devices_cooper_sleep(pHandle);

    return ui32ErrorStatus;
}


//*****************************************************************************
//
// Check the state of the IRQ pin.
//
//*****************************************************************************
uint32_t
am_devices_cooper_irq_read(void)
{
    return am_hal_gpio_input_read(AM_DEVICES_COOPER_IRQ_PIN);
}

//*****************************************************************************
//
// Check the state of the CLKREQ pin.
//
//*****************************************************************************
uint32_t
am_devices_cooper_clkreq_read(void* pHandle)
{
    return am_hal_gpio_input_read(AM_DEVICES_COOPER_CLKREQ_PIN);
}

//*****************************************************************************
//
// Set the 32M crystal frequency
// based on the tested values at customer side.
// set trim value smaller in case of negative frequency offset
// ui32TrimValue: default is 0x11C
//*****************************************************************************
uint32_t
am_devices_cooper_crystal_trim_set(void *pHandle, uint32_t ui32TrimValue)
{
    MCUCTRL->XTALHSTRIMS_b.XTALHSCAPTRIM = (ui32TrimValue & MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM_Msk) >> MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM_Pos;
    MCUCTRL->XTALHSTRIMS_b.XTALHSCAP2TRIM = (ui32TrimValue & MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM_Msk) >> MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM_Pos;

    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////
//
// SBL Driver
//
//
/////////////////////////////////////////////////////////////////////////////////

//*****************************************************************************
//
// Initialize the SPI.
//
//*****************************************************************************

//*****************************************************************************
//
// Read a packet from the SBL IOS.
//
//*****************************************************************************
bool iom_slave_read(void* pHandle, uint32_t* pBuf, uint32_t* psize)
{
    am_secboot_wired_msghdr_t* msg;
    uint32_t crc;
    am_devices_cooper_blocking_read(pHandle, pBuf, psize);
    // Verify the received data CRC
    msg = (am_secboot_wired_msghdr_t*)pBuf;
    am_hal_crc32((uint32_t)&msg->msgType, msg->length - sizeof(uint32_t), &crc);
    if ( crc != msg->crc32 )
    {
        return false;
    }
    else
    {
        return true;
    }
}

//*****************************************************************************
//
// Send a "HELLO" packet.
//
//*****************************************************************************
void send_hello(void* pHandle)
{
    am_secboot_wired_msghdr_t msg;
    msg.msgType = AM_SBL_HOST_MSG_HELLO;
    msg.length = sizeof(am_secboot_wired_msghdr_t);
    //
    // Compute CRC
    //
    //PRT_INFO("send_hello: sending bytes: %d.\n", msg.length );
    am_hal_crc32((uint32_t)&msg.msgType, msg.length - sizeof(uint32_t), &msg.crc32);
    am_devices_cooper_blocking_write(pHandle, AM_DEVICES_COOPER_RAW, (uint32_t*)&msg, sizeof(msg), true);
}

//*****************************************************************************
//
// Send a "UPDATE" packet.
//
//*****************************************************************************
void send_update(void* pHandle, uint32_t imgBlobSize)
{
    am_sbl_host_msg_update_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_UPDATE;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_update_t);
    msg.imageSize = imgBlobSize;
    // Check if we are downloading a newer FW versiion
    if ((gsSblUpdateState.ui32CooperFWImageVersion < g_sFwImage.version)
         || (gsSblUpdateState.ui32CooperVerRollBackConfig & 0x00000001))
    {
        msg.versionNumber = g_sFwImage.version;
    }
    else
    {
        msg.versionNumber = gsSblUpdateState.ui32CooperFWImageVersion;
    }
    msg.NumPackets = gsSblUpdateState.ui32TotalPackets + 1; // One addition packet as header will be a seperate packet
    msg.maxPacketSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    am_devices_cooper_blocking_write(pHandle, AM_DEVICES_COOPER_RAW, (uint32_t*)&msg, sizeof(msg), true);
}

//*****************************************************************************
//
// Send a "Data" packet.
//
//*****************************************************************************
void send_data(void* pHandle, uint32_t address, uint32_t size, uint32_t pktNumber)
{
    // reuse same buffer for receiving
    am_sbl_host_msg_data_t* msg = (am_sbl_host_msg_data_t*)gsSblUpdateState.pWorkBuf;
    msg->msgHdr.msgType = AM_SBL_HOST_MSG_DATA;
    msg->msgHdr.msgLength = sizeof(am_sbl_host_msg_data_t) + size;
    msg->packetNumber = pktNumber;
    memcpy((uint8_t*)msg->data, (uint8_t*)address, size);
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t) & (msg->msgHdr.msgType), msg->msgHdr.msgLength - sizeof(uint32_t), &msg->msgHdr.msgCrc);
    am_devices_cooper_blocking_write(pHandle, AM_DEVICES_COOPER_RAW, (uint32_t*)msg, (sizeof(am_sbl_host_msg_data_t) + size), true);
}

//*****************************************************************************
//
// Send a "Reset" packet.
//
//*****************************************************************************
void send_reset(void* pHandle)
{
    am_sbl_host_msg_reset_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_RESET;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_reset_t);
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    am_devices_cooper_blocking_write(pHandle, AM_DEVICES_COOPER_RAW, (uint32_t*)&msg, sizeof(msg), true);
}

//*****************************************************************************
//
// Send a "FW Continue  packet.
//
//*****************************************************************************
void send_fwContinue(void* pHandle)
{
    am_sbl_host_msg_fw_continue_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_FW_CONTINUE;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_fw_continue_t);
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    am_devices_cooper_blocking_write(pHandle, AM_DEVICES_COOPER_RAW, (uint32_t*)&msg, sizeof(msg), true);
}

//*****************************************************************************
//
// Update the state machine based on the image to download
//
//*****************************************************************************
static bool am_devices_cooper_sbl_update_state_data(uint32_t ui32updateType)
{
    // Pointer to the data to be updated
    am_devices_cooper_sbl_update_data_t* p_sUpdateImageData = NULL;
    if ( ui32updateType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW )
    {
        p_sUpdateImageData = &g_sFwImage;
    }
    else if ( ui32updateType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0 )
    {
        p_sUpdateImageData = &g_sInfo0PatchImage;
    }
    else if ( ui32updateType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1 )
    {
        p_sUpdateImageData = &g_sInfo1PatchImage;
    }
    else
    {
        return false;
    }
    // Check if the data is valid
    if (    (p_sUpdateImageData != NULL)                &&
            (p_sUpdateImageData->pImageAddress != 0 )    &&
            (p_sUpdateImageData->imageSize != 0 )       &&
            (p_sUpdateImageData->imageType == ui32updateType) )
    {
        // Load the INFO 0 Patch address
        gsSblUpdateState.pImageBuf          = p_sUpdateImageData->pImageAddress;
        // Image size
        gsSblUpdateState.ui32ImageSize      = p_sUpdateImageData->imageSize;
        // image type
        gsSblUpdateState.ui32ImageType      = p_sUpdateImageData->imageType;
        // Get the size of the data without headers
        gsSblUpdateState.ui32DataSize       = gsSblUpdateState.ui32ImageSize - AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE;
        // Get the start address of the data without headers
        gsSblUpdateState.pDataBuf           = gsSblUpdateState.pImageBuf + AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE;
        // Calculate number of packets
        gsSblUpdateState.ui32TotalPackets   = gsSblUpdateState.ui32DataSize / AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
        if (  (gsSblUpdateState.ui32DataSize % AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE) != 0 )
        {
            gsSblUpdateState.ui32TotalPackets++;
        }
        gsSblUpdateState.ui32PacketNumber = 0;
        return true;
    }

    return false;
}
//*****************************************************************************
//
// Initialize the Image Update state machine
//
//*****************************************************************************
uint32_t am_devices_cooper_image_update_init(void* pHandle, uint32_t* pWorkBuf)
{
    // Check for the input data validity
    if (pHandle != NULL)
    {
        // TODO: for Rev B; Authenticate image using RSA Public Key
        // Initialize state machine
        gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_INIT;
        // Load the image address
        gsSblUpdateState.pImageBuf          = NULL;
        // Image size
        gsSblUpdateState.ui32ImageSize      = 0;
        // image type
        gsSblUpdateState.ui32ImageType      = AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_NONE;
        // Get the size of the data without headers
        gsSblUpdateState.ui32DataSize       = 0;
        // Get the start address of the data without headers
        gsSblUpdateState.pDataBuf           = NULL;
        // Calculate number of packets
        gsSblUpdateState.ui32TotalPackets   = 0;
        // Initialize Packet number in progress
        gsSblUpdateState.ui32PacketNumber = 0;
        //
        // Save cooper device handle
        //
        gsSblUpdateState.pHandle = pHandle;
        //
        // Work buffer reuse non-blocking work buffer.
        //
        gsSblUpdateState.pWorkBuf = pWorkBuf;
        // State is ready to go. Reset the cooper device
        // TODO: Reset the Cooper device
        return 0;
    }
    else
    {
        // Return with error
        return 1;
    }
}

//*****************************************************************************
//
// Update Image
//
//*****************************************************************************
uint32_t am_devices_cooper_update_image(void)
{
    uint32_t     ui32dataPktSize = 0;
    uint32_t     ui32Size        = 0;
    uint32_t     ui32Ret         = AM_DEVICES_COOPER_SBL_STATUS_INIT;
    am_sbl_host_msg_status_t*    psStatusMsg;
    am_sbl_host_msg_ack_nack_t*  psAckMsg;
    switch (gsSblUpdateState.ui32SblUpdateState)
    {
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_INIT:
            //
            // Send the "HELLO" message to connect to the interface.
            //
            send_hello(gsSblUpdateState.pHandle);
            gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_HELLO;
            // Tell application that we are not done with SBL
            ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_HELLO:
            // Read the "STATUS" response from the IOS and check for CRC Error
            if ( iom_slave_read(gsSblUpdateState.pHandle, (uint32_t*)gsSblUpdateState.pWorkBuf, &ui32Size) == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_hello(gsSblUpdateState.pHandle);
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Check the status
                psStatusMsg = (am_sbl_host_msg_status_t*) (gsSblUpdateState.pWorkBuf);
                gsSblUpdateState.ui32CooperSblStatus = psStatusMsg->bootStatus;
                // Get the Cooper FW version
                if ( psStatusMsg->versionNumber == AM_DEVICES_COOPER_SBL_DEFAULT_FW_VERSION )
                {
                    gsSblUpdateState.ui32CooperFWImageVersion = 0;
                }
                else
                {
                    gsSblUpdateState.ui32CooperFWImageVersion = psStatusMsg->versionNumber;
                }
                am_util_stdio_printf("Cooper FW Ver = %d.%d\r\n", (psStatusMsg->versionNumber & 0xF00) >> 8, psStatusMsg->versionNumber & 0xFF);

                if (ui32Size == sizeof(am_sbl_host_msg_status_t))
                {
                    // Get the version rollback configuration
                    gsSblUpdateState.ui32CooperVerRollBackConfig = psStatusMsg->verRollBackStatus;

#if (SBL_DEBUG_LOG_ON == 1)
                    if ( psStatusMsg->verRollBackStatus == AM_DEVICES_COOPER_SBL_STAT_VER_ROLL_BACK_EN )
                    {
                        am_util_stdio_printf("Version RollBack Enabled \n");

                    }
                    else if ( psStatusMsg->verRollBackStatus == AM_DEVICES_COOPER_SBL_STAT_VER_ROLL_BACK_DBL )
                    {
                        am_util_stdio_printf("Version RollBack Disabled \n");
                    }
                    else
                    {
                        am_util_stdio_printf("Version RollBack Config invalid !!! \n");
                    }

                    am_util_stdio_printf("Cooper Chip ID Word 0 =0x%x\r\n", psStatusMsg->copperChpIdWord0);
                    am_util_stdio_printf("Cooper Chip ID Word 1 =0x%x\r\n", psStatusMsg->copperChpIdWord1);
#endif
                }
                else
                {
                    gsSblUpdateState.ui32CooperVerRollBackConfig = 0x0;
                }
                #if (SBL_DEBUG_LOG_ON == 1)
                am_util_stdio_printf("SBL Cooper Status:  0x%x\r\n", sbl_status);
                am_util_stdio_printf("bootStatus 0x%x\r\n", psStatusMsg->bootStatus);
                #endif
                // check if the Boot Status is success
                if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_SUCCESS )
                {
                    // Check if we have some FW available
                    if (  am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW) == true )
                    {
                        // When rollback is enabled, we'll upgrade firmware regardless of firmware version.
                        // When rollback is disabled, we'll only upgrade when a to-be-downloaded firmware's version
                        // is higher the existing one on BLE controller.
                        if ( (psStatusMsg->versionNumber < g_sFwImage.version )
                             || (gsSblUpdateState.ui32CooperVerRollBackConfig & 0x00000001))

                        {
                            ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_FW;
                            am_util_stdio_printf("Received new Cooper FW version = %d.%d Going for upgrade\r\n", (g_sFwImage.version & 0xF00) >> 8, g_sFwImage.version & 0xFF);
                        }
                    }
                    // If we don't have any FW or any newer FW then continue with the current FW in Cooper
                    if ( ui32Ret != AM_DEVICES_COOPER_SBL_STATUS_UPDATE_FW )
                    {
                        // We don't have any other FW, so continue with one already there is Cooper device
                        gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK;
                        // Not done yet
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;

                        #if (SBL_DEBUG_LOG_ON == 1)
                        am_util_stdio_printf(" Cooper FW Auth Passed, Continue with FW \n");
                        #endif
                        // Send the command to continue to FW
                        send_fwContinue(gsSblUpdateState.pHandle);
                    }
                }
                else if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_FW_UPDATE_REQ )
                {
                    #if (SBL_DEBUG_LOG_ON == 1)
                    am_util_stdio_printf("Cooper SBL Status : Request for FW \r\n");
                    #endif
                    if (  am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW) == true )
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_FW;
                    }
                    else
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL;
                    }
                }
                else if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_INFO0_UPDATE_REQ )
                {
                    #if (SBL_DEBUG_LOG_ON == 1)
                    am_util_stdio_printf("Cooper SBL Status : Request for INFO 0 \r\n");
                    #endif
                    if ( am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0) == true )
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_INFO_0;
                    }
                    else
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL;
                    }
                }
                else if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_INFO1_UPDATE_REQ )
                {
                    #if (SBL_DEBUG_LOG_ON == 1)
                    am_util_stdio_printf("Cooper SBL Status : Request for INFO 1 \r\n");
                    #endif
                    if ( am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1) == true )
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_INFO_1;
                    }
                    else
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL;
                    }
                }
                else
                {
                    #if (SBL_DEBUG_LOG_ON == 1)
                    am_util_stdio_printf("Cooper SBL Status : Failed \r\n");
                    #endif
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
            }
            if (  (ui32Ret == AM_DEVICES_COOPER_SBL_STATUS_OK) || (ui32Ret == AM_DEVICES_COOPER_SBL_STATUS_FAIL) ||
                  (gsSblUpdateState.ui32SblUpdateState == AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK) )
            {
                // Do nothing
            }
            else
            {
                //Update the state machine
                gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_UPDATE;
                // Send the update message
                send_update(gsSblUpdateState.pHandle, gsSblUpdateState.ui32ImageSize);
                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
            }
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_UPDATE:
            // Read the "STATUS" response from the IOS and check for CRC Error
            if ( iom_slave_read(gsSblUpdateState.pHandle, (uint32_t*)gsSblUpdateState.pWorkBuf, &ui32Size) == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_update(gsSblUpdateState.pHandle, gsSblUpdateState.ui32ImageSize);
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Get the response status
                psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
                // Process the response
                if ( (psAckMsg->msgHdr.msgType == AM_SBL_HOST_MSG_ACK) && (NULL != gsSblUpdateState.pImageBuf))
                {
                    // Save the status
                    gsSblUpdateState.ui32CooperSblStatus = psAckMsg->status;
                    // Change the state
                    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_DATA;
                    // Send the Encrypted image header - first 64 bytes
                    send_data(gsSblUpdateState.pHandle, (uint32_t)gsSblUpdateState.pImageBuf,
                            AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE, gsSblUpdateState.ui32PacketNumber);
                    am_util_stdio_printf("Cooper upgrade in progress, wait...\r\n");
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
                else
                {
                    am_util_stdio_printf("Update Failed !!! \r\n");
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
            }
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_DATA:
            // Read the "STATUS" response from the IOS.
            if ( iom_slave_read(gsSblUpdateState.pHandle, (uint32_t*)gsSblUpdateState.pWorkBuf, &ui32Size) == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    if ( gsSblUpdateState.ui32PacketNumber == 0 )
                    {
                        // Send the Encrypted image header - first 64 bytes
                        send_data(gsSblUpdateState.pHandle, (uint32_t)gsSblUpdateState.pImageBuf,
                                  AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE, gsSblUpdateState.ui32PacketNumber);
                    }
                    else
                    {
                        // Reset the packet counters to the previous ones, to resend the packet
                        //gsSblUpdateState.ui32TotalPackets++;
                        // increment the packet number as we have already sent the header
                        //gsSblUpdateState.ui32PacketNumber--;
                        //Check if this is the last packet - Increase by one as we have already decremented after TX
                        if (  (gsSblUpdateState.ui32TotalPackets + 1) == 1 )
                        {
                            // Get the size of the leftover data
                            ui32dataPktSize = gsSblUpdateState.ui32DataSize % AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            if (ui32dataPktSize == 0)
                            {
                                ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            }
                        }
                        else
                        {
                            ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                        }
                        // Resend the same packet - Need to decrement the packet numbers as those are already incremented
                        send_data(gsSblUpdateState.pHandle, (uint32_t) gsSblUpdateState.pDataBuf + ( (gsSblUpdateState.ui32PacketNumber - 1) * AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE),
                                  ui32dataPktSize, gsSblUpdateState.ui32PacketNumber);
                    }
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Get the response status
                psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
                // Save the status
                gsSblUpdateState.ui32CooperSblStatus = psAckMsg->status;
                if (  (psAckMsg->srcMsgType == AM_SBL_HOST_MSG_DATA ) || (psAckMsg->srcMsgType == AM_SBL_HOST_MSG_UPDATE_STATUS) )
                {
                    if (  (psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SUCCESS) || (psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SEQ) )
                    {
                        if ( gsSblUpdateState.ui32TotalPackets > 0 )
                        {
                            //Check if this is the last packet
                            if ( gsSblUpdateState.ui32TotalPackets == 1 )
                            {
                                // Get the size of the left over data
                                ui32dataPktSize = gsSblUpdateState.ui32DataSize % AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                                if (ui32dataPktSize == 0)
                                {
                                    ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                                }
                            }
                            else
                            {
                                ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            }
                            send_data(gsSblUpdateState.pHandle, (uint32_t) gsSblUpdateState.pDataBuf + (gsSblUpdateState.ui32PacketNumber * AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE),
                                      ui32dataPktSize, gsSblUpdateState.ui32PacketNumber + 1);
                            gsSblUpdateState.ui32TotalPackets--;
                            // increment the packet number as we have already sent the header
                            gsSblUpdateState.ui32PacketNumber++;
                            ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                        }
                        else
                        {
                            if ( psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SUCCESS )
                            {
                                // If FW is updated successfuly, then jump to BLE image
                                if ( gsSblUpdateState.ui32ImageType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW )
                                {
                                    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK;
                                    // Not done yet
                                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                                    // Send the command to continue to FW
                                    send_fwContinue(gsSblUpdateState.pHandle);
                                    // If INFO 0 or INFO 1 is updated successfully, the apply send reset
                                }
                                else
                                {
                                    // TODO: Check if this can be a timing problem as Cooper SPI might have already raised
                                    // the IRQ
                                    // We are assuming Cooper is in Reset state, and waiting for Hello
                                    // so start the state machine again to send the Hello packet
                                    // Change the state machine state
                                    //gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_INIT;
                                    // Done done yet
                                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_OK;
                                }
                            }
                            else
                            {
                                #if (SBL_DEBUG_LOG_ON == 1)
                                am_util_stdio_printf("Update fails status=0x%x\r\n", psAckMsg->status);
                                #endif
                                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                            }
                        }
                    }
                    else
                    {
                        #if (SBL_DEBUG_LOG_ON == 1)
                        am_util_stdio_printf("Update fails status=0x%x\r\n", psAckMsg->status);
                        #endif
                        // We have received NACK
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                    }
                }
                else
                {
                    // Wrong Response type
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
            }
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK:
        {
            // Read the "STATUS" response from the IOS and check for CRC Error
            if ( iom_slave_read(gsSblUpdateState.pHandle, (uint32_t*)gsSblUpdateState.pWorkBuf, &ui32Size) == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_fwContinue(gsSblUpdateState.pHandle);
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
            }
            // Get the response status
            psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
            // Save the status
            gsSblUpdateState.ui32CooperSblStatus = psAckMsg->status;
            if ( psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SUCCESS )
            {
                // FW has gone to BLE, end the SBL driver state machine
                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_OK;
            }
            else
            {
                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
            }
        }
        break;
        default:
            // Bad state, update the state machine
            break;
    }
    return ui32Ret;
}

bool am_devices_cooper_get_FwImage(am_devices_cooper_sbl_update_data_t *pFwImage )
{
    if (pFwImage != NULL)
    {
        memcpy(&g_sFwImage, pFwImage, sizeof(am_devices_cooper_sbl_update_data_t));
    }

    return (pFwImage != NULL);
}

bool am_devices_cooper_get_info1_patch(am_devices_cooper_sbl_update_data_t *pInfo1Image)
{
    if (pInfo1Image != NULL)
    {
        memcpy(&g_sInfo1PatchImage, pInfo1Image, sizeof(am_devices_cooper_sbl_update_data_t));
    }

    return (pInfo1Image != NULL);
}

bool am_devices_cooper_get_info0_patch(am_devices_cooper_sbl_update_data_t *pInfo0Image)
{
    if (pInfo0Image != NULL)
    {
        memcpy(&g_sInfo0PatchImage, pInfo0Image, sizeof(am_devices_cooper_sbl_update_data_t));
    }

    return (pInfo0Image != NULL);
}

//*****************************************************************************
//
// Reset the BLE controller and check if there's request to update
//
//*****************************************************************************
uint32_t am_devices_cooper_reset_with_sbl_check(void* pHandle, am_devices_cooper_config_t* pDevConfig)
{
    uint32_t sbl_status = 0;
    am_devices_cooper_reset();
    am_devices_cooper_image_update_init(pHandle, pDevConfig->pNBTxnBuf);
    sbl_status = AM_DEVICES_COOPER_SBL_STATUS_INIT;
    sbl_status = am_devices_cooper_update_image();
    while( (sbl_status != AM_DEVICES_COOPER_SBL_STATUS_OK) && ( sbl_status != AM_DEVICES_COOPER_SBL_STATUS_FAIL) )
    {
        while (am_devices_cooper_irq_read() == 0)
        {
            am_hal_delay_us(50);
        }
        sbl_status = am_devices_cooper_update_image();
    }
    //
    // Return the status.
    //
    if (sbl_status == AM_DEVICES_COOPER_SBL_STATUS_OK)
    {
        // need to wait a bit to jump from SBL to Cooper application firmware
        am_util_delay_ms(10);
        am_util_stdio_printf("SBL Done\r\n");
        return AM_DEVICES_COOPER_STATUS_SUCCESS;
    }
    else
    {
        // free up resource that won't be used.
        am_devices_cooper_term(pHandle);
        am_util_stdio_printf("SBL Cooper Error 0x%x\r\n", sbl_status);
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }
}

//*****************************************************************************
//
// API to disable the BLE controller's firmware rollback version (Enabled in default)
// Should be called as the very last step during manufacturing, after it done,
// the BLE controller will reset.
//
//*****************************************************************************
uint32_t am_devices_cooper_disable_rollback(void* pHandle, am_devices_cooper_config_t* pDevConfig)
{
    const unsigned char info_1_enc_patch_disable_ver_rollBack_bin[] =
    {
        0x80, 0x00, 0x30, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x94, 0xfd, 0x5c, 0x6c,
        0x3b, 0x83, 0x36, 0x18, 0x73, 0x08, 0xa0, 0x95, 0xf5, 0x0b, 0xb4, 0xcd,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x35, 0x4c, 0x47, 0x0f, 0x16, 0xa3, 0x4d, 0xb7, 0x71, 0xf7, 0x6c, 0x8d,
        0x92, 0x63, 0x20, 0xac, 0xf5, 0x3e, 0xfd, 0xd6, 0xea, 0x3f, 0xcc, 0x4d,
        0x8a, 0xf6, 0x4e, 0xd6, 0x7f, 0x01, 0x74, 0xce, 0x0d, 0x95, 0x9a, 0x35,
        0xcc, 0xf2, 0x41, 0x3c, 0x42, 0x46, 0x78, 0x2d, 0x61, 0xaa, 0x87, 0xcb,
        0x1e, 0x5e, 0x22, 0x54, 0x5a, 0xcd, 0xee, 0x49, 0xab, 0xf6, 0xb9, 0x9e,
        0x16, 0xb4, 0x09, 0x08, 0x93, 0xe6, 0x41, 0x05, 0x41, 0x7a, 0xd4, 0x1d,
        0xf1, 0x9c, 0x72, 0xb2, 0x57, 0xfc, 0x25, 0xb7
    };
    uint8_t hci_vsc_write_info1_signature_cmd[1 + 2 + 1 + 4] = {0x01, 0x75, 0xFC, 0x04, AM_DEVICES_COOPER_SBL_INFO_1_PATCH_AVAILABLE_SIGN};
    uint32_t ui32ErrorStatus;

    // Update info1 image information to SBL
    g_sInfo1PatchImage.pImageAddress = (uint8_t*)info_1_enc_patch_disable_ver_rollBack_bin;
    g_sInfo1PatchImage.imageSize = sizeof(info_1_enc_patch_disable_ver_rollBack_bin);

    // For info 1 patching
    // write HCI command to trigger Cooper to reboot for SBL to do download.
    ui32ErrorStatus = am_devices_cooper_blocking_write(pHandle, AM_DEVICES_COOPER_RAW,
                                                        (uint32_t*)&hci_vsc_write_info1_signature_cmd,
                                                        sizeof(hci_vsc_write_info1_signature_cmd), true);
    if ( ui32ErrorStatus != AM_DEVICES_COOPER_STATUS_SUCCESS )
    {
        am_util_stdio_printf("Write to BLE Controller failed\n");
        return ui32ErrorStatus;
    }

    am_util_delay_ms(100);

    // reset Cooper to get SBL to update info1
    ui32ErrorStatus = am_devices_cooper_reset_with_sbl_check(pHandle, pDevConfig);
    return ui32ErrorStatus;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
