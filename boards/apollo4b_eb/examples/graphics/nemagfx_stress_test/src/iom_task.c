//*****************************************************************************
//
//! @file iom_task.c
//!
//! @brief Task to handle iom operations.
//!
//! AM_DEBUG_PRINTF
//! If enabled, debug messages will be sent over ITM.
//!
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

#include "nemagfx_stress_test.h"

#ifndef BAREMETAL

#define     IOM_MODULE          0
#define     USE_SPI             1   // 0 = I2C, 1 = SPI
#define     I2C_ADDR            0x10
// How much data to read from Slave before ending the test
#define     MAX_SIZE            10000

#define     XOR_BYTE            0
#define     EMPTY_BYTE          0xEE

typedef enum
{
    AM_IOSTEST_CMD_START_DATA    = 0,
    AM_IOSTEST_CMD_STOP_DATA     = 1,
    AM_IOSTEST_CMD_ACK_DATA      = 2,
} AM_IOSTEST_CMD_E;

#define IOSOFFSET_WRITE_INTEN       0xF8
#define IOSOFFSET_WRITE_INTCLR      0xFA
#define IOSOFFSET_WRITE_CMD         0x80
#define IOSOFFSET_READ_INTSTAT      0x79
#define IOSOFFSET_READ_FIFO         0x7F
#define IOSOFFSET_READ_FIFOCTR      0x7C

#define AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK  1

#define HANDSHAKE_PIN            10

//*****************************************************************************
//
// Global message buffer for the IO master.
//
//*****************************************************************************
#define AM_TEST_RCV_BUF_SIZE    1024 // Max Size we can receive is 1023
uint8_t g_pui8RcvBuf[AM_TEST_RCV_BUF_SIZE];
volatile uint32_t g_startIdx = 0;
volatile bool bIosInt = false;

void *g_IOMHandle;
//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
static am_hal_iom_config_t g_sIOMSpiConfig =
{
    .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
//    .ui32ClockFreq = AM_HAL_IOM_96MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_48MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_24MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_16MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_12MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_8MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_6MHZ,
    .ui32ClockFreq = AM_HAL_IOM_4MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_3MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_2MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_1_5MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_1MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_750KHZ,
//    .ui32ClockFreq = AM_HAL_IOM_500KHZ,
//    .ui32ClockFreq = AM_HAL_IOM_400KHZ,
//    .ui32ClockFreq = AM_HAL_IOM_375KHZ,
//    .ui32ClockFreq = AM_HAL_IOM_250KHZ,
//    .ui32ClockFreq = AM_HAL_IOM_125KHZ,
//    .ui32ClockFreq = AM_HAL_IOM_100KHZ,
//    .ui32ClockFreq = AM_HAL_IOM_50KHZ,
//    .ui32ClockFreq = AM_HAL_IOM_10KHZ,
    .eSpiMode = AM_HAL_IOM_SPI_MODE_0,
};

#define MAX_SPI_SIZE    1023

static am_hal_iom_config_t g_sIOMI2cConfig =
{
    .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq  = AM_HAL_IOM_1MHZ,
};

#define MAX_I2C_SIZE   255

const am_hal_gpio_pincfg_t g_AM_BSP_GPIO_HANDSHAKE =
{
    .GP.cfg_b.uFuncSel       = AM_HAL_PIN_10_GPIO,
    .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .GP.cfg_b.eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
    .GP.cfg_b.eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
};

void iom_slave_read(bool bSpi, uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 1;
#if defined(AM_PART_APOLLO4B)
    Transaction.ui64Instr = offset;
#else
    Transaction.ui32Instr = offset;
#endif
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32RxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    if ( bSpi )
    {
        Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
    }
    else
    {
        Transaction.uPeerInfo.ui32I2CDevAddr = I2C_ADDR;
    }
    am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
}

void iom_slave_write(bool bSpi, uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 1;
#if defined(AM_PART_APOLLO4B)
    Transaction.ui64Instr = offset;
#else
    Transaction.ui32Instr = offset;
#endif
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32TxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    if ( bSpi )
    {
        Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
    }
    else
    {
        Transaction.uPeerInfo.ui32I2CDevAddr = I2C_ADDR;
    }
    am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
}
static void iom_set_up(uint32_t iomModule, bool bSpi)
{
    uint32_t ioIntEnable = AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK;

    //
    // Initialize the IOM.
    //
    am_hal_iom_initialize(iomModule, &g_IOMHandle);

    am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);

    if ( bSpi )
    {
        //
        // Set the required configuration settings for the IOM.
        //
        am_hal_iom_configure(g_IOMHandle, &g_sIOMSpiConfig);

        //
        // Configure the IOM pins.
        //
        am_bsp_iom_pins_enable(iomModule, AM_HAL_IOM_SPI_MODE);
    }
    else
    {
        //
        // Set the required configuration settings for the IOM.
        //
        am_hal_iom_configure(g_IOMHandle, &g_sIOMI2cConfig);

        //
        // Configure the IOM pins.
        //
        am_bsp_iom_pins_enable(iomModule, AM_HAL_IOM_I2C_MODE);
    }

    //
    // Enable all the interrupts for the IOM module.
    //
    //am_hal_iom_InterruptEnable(g_IOMHandle, 0xFF);
    //am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER0);

    //
    // Enable the IOM.
    //
    am_hal_iom_enable(g_IOMHandle);
    am_hal_gpio_pinconfig(HANDSHAKE_PIN, g_AM_BSP_GPIO_HANDSHAKE);



    uint32_t IntNum = HANDSHAKE_PIN;
    am_hal_gpio_state_write(HANDSHAKE_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    // Set up the host IO interrupt
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_INT_CHANNEL_0, (am_hal_gpio_mask_t*)&IntNum);
    // Register handler for IOS => IOM interrupt
//    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, HANDSHAKE_PIN,
//                                    (am_hal_gpio_handler_t)hostint_handler, NULL);
//    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
//                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
//                                  (void *)&IntNum);
    NVIC_EnableIRQ(GPIO0_001F_IRQn);

    // Set up IOCTL interrupts
    // IOS ==> IOM
    iom_slave_write(bSpi, IOSOFFSET_WRITE_INTEN, &ioIntEnable, 1);
}

//*****************************************************************************
//
// iom task handle.
//
//*****************************************************************************
TaskHandle_t IomTaskHandle;

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
IomTask(void *pvParameters)
{
    uint32_t iom = IOM_MODULE;
    bool bSpi = USE_SPI;
    uint32_t data;

    iom_set_up(iom, bSpi);

    while(1)
    {
        data = 0xA5;
        am_util_stdio_printf("iom sends two bytes on SPI, 0x80 0xA5.\r\n");
        iom_slave_write(bSpi, IOSOFFSET_WRITE_CMD, &data, 1);
        vTaskDelay(100);
    }
}
#endif