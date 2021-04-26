//*****************************************************************************
//
//! @file i2s_loopback.c
//!
//! @brief An example to show basic I2S operation.
//!
//! Purpose: This example enables the I2S interfaces to loop back data from
//! each other.  Either I2S0 or I2S1 can be selected as the master.
//! The required pin connections are as follows.
//! Apollo4 (BGA):
//! J9-2 I2S1DIN  to J9-16 I2S0DOUT
//! J9-4 I2S1WS   to J9-14 I2S0WS
//! J9-6 I2S1DOUT to J9-12 I2S0DIN
//! J9-8 I2S1CLK  to J11-7 I2S0CK
//!
//! Apollo4 Blue (SIP):
//! J9-1 I2S1DIN  to J9-8  I2S0DOUT
//! J9-2 I2S1WS   to J9-7  I2S0WS
//! J9-3 I2S1DOUT to J9-6  I2S0DIN
//! J9-4 I2S1CLK  to J11-7 I2S0CK
//!
//! Printing takes place over the ITM at 1M Baud.
//!
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
#include <arm_math.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Select master
//
//*****************************************************************************
#define     USE_I2S1_MASTER             0 // 0: master = I2S0; 1: master = I2S1

//*****************************************************************************
//
// Non-selectable definitions
//
//*****************************************************************************
#define     I2S_MODULE_MASTER           USE_I2S1_MASTER   // I2S master
#if I2S_MODULE_MASTER == 0
#define     I2S_MODULE_SLAVE            1   // I2S slave = I2S1
#else
#define     I2S_MODULE_SLAVE            0   // I2S slave = I2S0
#endif

#define BUFFER_SIZE_BYTES               1024

#if USE_I2S1_MASTER == 0                // If master = I2S0...

#define I2S_DATA_IN_GPIO_FUNC           AM_HAL_PIN_14_I2S0_SDIN
#define I2S_DATA_IN_GPIO_PIN            14
#define I2S_DATA_OUT_GPIO_FUNC          AM_HAL_PIN_12_I2S0_SDOUT
#define I2S_DATA_OUT_GPIO_PIN           12
#define I2S_CLK_GPIO_FUNC               AM_HAL_PIN_11_I2S0_CLK
#define I2S_CLK_GPIO_PIN                11
#define I2S_WS_GPIO_FUNC                AM_HAL_PIN_13_I2S0_WS
#define I2S_WS_GPIO_PIN                 13

#define I2S_SLAVE_DATA_IN_GPIO_FUNC     AM_HAL_PIN_19_I2S1_SDIN
#define I2S_SLAVE_DATA_IN_GPIO_PIN      19
#define I2S_SLAVE_DATA_OUT_GPIO_FUNC    AM_HAL_PIN_17_I2S1_SDOUT
#define I2S_SLAVE_DATA_OUT_GPIO_PIN     17
#define I2S_SLAVE_CLK_GPIO_FUNC         AM_HAL_PIN_16_I2S1_CLK
#define I2S_SLAVE_CLK_GPIO_PIN          16
#define I2S_SLAVE_WS_GPIO_FUNC          AM_HAL_PIN_18_I2S1_WS
#define I2S_SLAVE_WS_GPIO_PIN           18

#else                                   // If master = I2S1...

#define I2S_DATA_IN_GPIO_FUNC           AM_HAL_PIN_19_I2S1_SDIN
#define I2S_DATA_IN_GPIO_PIN            19
#define I2S_DATA_OUT_GPIO_FUNC          AM_HAL_PIN_17_I2S1_SDOUT
#define I2S_DATA_OUT_GPIO_PIN           17
#define I2S_CLK_GPIO_FUNC               AM_HAL_PIN_16_I2S1_CLK
#define I2S_CLK_GPIO_PIN                16
#define I2S_WS_GPIO_FUNC                AM_HAL_PIN_18_I2S1_WS
#define I2S_WS_GPIO_PIN                 18

#define I2S_SLAVE_DATA_IN_GPIO_FUNC     AM_HAL_PIN_14_I2S0_SDIN
#define I2S_SLAVE_DATA_IN_GPIO_PIN      14
#define I2S_SLAVE_DATA_OUT_GPIO_FUNC    AM_HAL_PIN_12_I2S0_SDOUT
#define I2S_SLAVE_DATA_OUT_GPIO_PIN     12
#define I2S_SLAVE_CLK_GPIO_FUNC         AM_HAL_PIN_11_I2S0_CLK
#define I2S_SLAVE_CLK_GPIO_PIN          11
#define I2S_SLAVE_WS_GPIO_FUNC          AM_HAL_PIN_13_I2S0_WS
#define I2S_SLAVE_WS_GPIO_PIN           13

#endif

//*****************************************************************************
//
// I2S interrupt configuration.
//
//*****************************************************************************
//
//! I2S interrupts.
//
static const IRQn_Type i2s_interrupts[] =
{
    I2S0_IRQn,
    I2S1_IRQn
};

void *pI2SHandle;
void *pI2SSlaveHandle;

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
I2S0_Type* g_I2S0;
I2S0_Type* g_I2S1;

AM_SHARED_RW uint32_t g_ui32I2SRxDataBuffer[BUFFER_SIZE_BYTES / 4 + 4];
AM_SHARED_RW uint32_t g_ui32I2STxDataBuffer[BUFFER_SIZE_BYTES / 4 + 4];
AM_SHARED_RW uint32_t g_ui32I2SRxDataBuffer_slave[BUFFER_SIZE_BYTES / 4 + 4];
AM_SHARED_RW uint32_t g_ui32I2STxDataBuffer_slave[BUFFER_SIZE_BYTES / 4 + 4];

uint32_t g_ui32I2SDmaCpl[8] =
{
    0, //Master TX.
    0, //Master RX.
    0, //Slave Tx.
    0, //Slave Rx.
    0  //Success or Fail.
};
uint32_t g_ui32I2S0_RXCpl = 0xA;
uint32_t g_ui32I2S0_TXCpl = 0xB;
uint32_t g_ui32I2S1_RXCpl = 0xC;
uint32_t g_ui32I2S1_TXCpl = 0xD;

//
// Programmer Reference setting.
//
static am_hal_i2s_io_signal_t g_sI2SIOConfig =
{
    .eFyncCpol = AM_HAL_I2S_IO_FSYNC_CPOL_HIGH,
    .eTxCpol = AM_HAL_I2S_IO_TX_CPOL_FALLING,
    .eRxCpol = AM_HAL_I2S_IO_RX_CPOL_RISING
};

static am_hal_i2s_data_format_t g_sI2SDataConfig =
{
    .ePhase = AM_HAL_I2S_DATA_PHASE_SINGLE,
    .eDataDelay = 0x1,
    .eDataJust = AM_HAL_I2S_DATA_JUSTIFIED_LEFT,
    .eChannelLenPhase1 = AM_HAL_I2S_FRAME_32BITS_WDLEN,
    .eChannelLenPhase2 = AM_HAL_I2S_FRAME_32BITS_WDLEN,
    .eSampleLenPhase1 = AM_HAL_I2S_FRAME_24BITS_WDLEN,
    .eSampleLenPhase2 = AM_HAL_I2S_FRAME_24BITS_WDLEN
};

static am_hal_i2s_config_t g_sI2SConfig =
{
    .eClock               = eAM_HAL_I2S_CLKSEL_HFRC_6MHz,
    .eDiv3                = 0,
    .eMode                = AM_HAL_I2S_IO_MODE_MASTER,
    .eXfer                = AM_HAL_I2S_XFER_RXTX,
    .ui32ChnNumber        = 2,
    .eData                = &g_sI2SDataConfig,
    .eIO                  = &g_sI2SIOConfig
};

static am_hal_i2s_config_t g_sI2SConfig_slave =
{
    .eClock               = eAM_HAL_I2S_CLKSEL_HFRC_6MHz,
    .eDiv3                = 0,
    .eASRC                = 0,
    .eMode                = AM_HAL_I2S_IO_MODE_SLAVE,
    .eXfer                = AM_HAL_I2S_XFER_RXTX,
    .ui32ChnNumber        = 2,
    .eData                = &g_sI2SDataConfig,
    .eIO                  = &g_sI2SIOConfig
};

//
// Transfer settings.
//
static am_hal_i2s_transfer_t sTransfer =
{
    .ui32RxTotalCount = BUFFER_SIZE_BYTES / 4,
    .ui32RxTargetAddr = (uint32_t)g_ui32I2SRxDataBuffer,
    .ui32TxTotalCount = BUFFER_SIZE_BYTES / 4,
    .ui32TxTargetAddr = (uint32_t)g_ui32I2STxDataBuffer
};

static am_hal_i2s_transfer_t sTransfer_slave =
{
    .ui32RxTotalCount = BUFFER_SIZE_BYTES / 4,
    .ui32RxTargetAddr = (uint32_t)g_ui32I2SRxDataBuffer_slave,
    .ui32TxTotalCount = BUFFER_SIZE_BYTES / 4,
    .ui32TxTargetAddr = (uint32_t)g_ui32I2STxDataBuffer_slave
};

//*****************************************************************************
//
// I2S helper function.
//
//*****************************************************************************
static bool check_i2s_data(uint32_t rxtx_size, uint32_t* rx_databuf,          \
                           uint32_t* tx_databuf)
{
    int i, index_0 = 0;

    //
    // Find the number 1 in the slave Rx buffer, and return the index of '0'.
    // The TX data starts from '0'.
    // Rx will delay 4 samples in full duplex mode.
    //
    for ( i = 0; i < rxtx_size; i++ )
    {
        if ( (rx_databuf[i] & 0x1) == 1 )
        {
            index_0 = i - 1;
            break;
        }
    }
    for ( i = 0; i < rxtx_size-index_0; i++ )
    {
        if ( rx_databuf[i + index_0] != tx_databuf[i] )
        {
            return false;
        }
    }
    return true;
}

//*****************************************************************************
//
// I2S0 interrupt handler.
//
//*****************************************************************************
void
am_dspi2s0_isr()
{
    uint32_t ui32Status;

//
// If master = I2S0, this is the I2S0 master handler.
//
#if USE_I2S1_MASTER == 0
    am_hal_i2s_interrupt_status_get(pI2SHandle, &ui32Status, true);
    am_hal_i2s_interrupt_clear(pI2SHandle, ui32Status);

    if (ui32Status & AM_HAL_I2S_INT_RXDMACPL)
    {
        uint32_t ui32DMAStatus;
        am_hal_i2s_dma_status_get(pI2SHandle, &ui32DMAStatus, AM_HAL_I2S_XFER_RX);

        //
        // The RX/TX DMACPL interrupt asserts when the programmed DMA completes,
        // or end with an errorcondition.
        //
        if ( ui32DMAStatus & AM_HAL_I2S_STAT_DMA_RX_ERR )
        {
          // Clear DMAERR bit
          am_hal_i2s_dma_error(pI2SHandle, AM_HAL_I2S_XFER_RX);
        }
        g_ui32I2SDmaCpl[1] = 1;
        g_ui32I2S0_RXCpl = 1;
    }

    if (ui32Status & AM_HAL_I2S_INT_TXDMACPL)
    {
        uint32_t ui32DMAStatus;
        am_hal_i2s_dma_status_get(pI2SHandle, &ui32DMAStatus, AM_HAL_I2S_XFER_TX);
        if ( ui32DMAStatus & AM_HAL_I2S_STAT_DMA_RX_ERR )
        {
          // Clear DMAERR bit.
          am_hal_i2s_dma_error(pI2SHandle, AM_HAL_I2S_XFER_TX);
        }
        g_ui32I2SDmaCpl[0] = 1;
        g_ui32I2S0_TXCpl = 1;
    }

    if (ui32Status & AM_HAL_I2S_INT_IPB)
    {
      am_hal_i2s_ipb_interrupt_service(pI2SHandle);
    }

//
// If master = I2S1, this is the I2S0 slave handler.
//
#else
    am_hal_i2s_interrupt_status_get(pI2SSlaveHandle, &ui32Status, true);
    am_hal_i2s_interrupt_clear(pI2SSlaveHandle, ui32Status);
    if (ui32Status & AM_HAL_I2S_INT_RXDMACPL)
    {
        uint32_t ui32DMAStatus;
        am_hal_i2s_dma_status_get(pI2SSlaveHandle, &ui32DMAStatus,             \
                                  AM_HAL_I2S_XFER_RX);
        //
        // The RX/TX DMACPL interrupt asserts when the programmed DMA completes,
        // or ends with an error condition.
        //
        if ( ui32DMAStatus & AM_HAL_I2S_STAT_DMA_RX_ERR )
        {
          //
          // Clear DMAERR bit.
          //
          am_hal_i2s_dma_error(pI2SSlaveHandle, AM_HAL_I2S_XFER_RX);
        }
        g_ui32I2SDmaCpl[3] = 1;
        g_ui32I2S1_RXCpl = 1;
    }

    if (ui32Status & AM_HAL_I2S_INT_TXDMACPL)
    {
        uint32_t ui32DMAStatus;
        am_hal_i2s_dma_status_get(pI2SSlaveHandle, &ui32DMAStatus,             \
                                  AM_HAL_I2S_XFER_TX);
        if ( ui32DMAStatus & AM_HAL_I2S_STAT_DMA_RX_ERR )
        {
          //
          // Clear DMAERR bit.
          //
          am_hal_i2s_dma_error(pI2SSlaveHandle, AM_HAL_I2S_XFER_TX);
        }
        g_ui32I2SDmaCpl[2] = 1;
        g_ui32I2S1_TXCpl = 1;
    }

    if (ui32Status & AM_HAL_I2S_INT_IPB)
    {
      am_hal_i2s_ipb_interrupt_service(pI2SSlaveHandle);
    }
#endif
}


//*****************************************************************************
//
// I2S1 interrupt handler.
//
//*****************************************************************************
void
am_dspi2s1_isr()
{
    uint32_t ui32Status;

//
// If master = I2S1, this is the I2S1 master handler.
//
#if USE_I2S1_MASTER == 1
    am_hal_i2s_interrupt_status_get(pI2SHandle, &ui32Status, true);
    am_hal_i2s_interrupt_clear(pI2SHandle, ui32Status);

    if (ui32Status & AM_HAL_I2S_INT_RXDMACPL)
    {
        uint32_t ui32DMAStatus;
        am_hal_i2s_dma_status_get(pI2SHandle, &ui32DMAStatus, AM_HAL_I2S_XFER_RX);
        //
        // The RX/TX DMACPL interrupt asserts when the programmed DMA completes,
        // or ends with an error condition.
        //
        if ( ui32DMAStatus & AM_HAL_I2S_STAT_DMA_RX_ERR )
        {
          //
          // Clear DMAERR bit.
          //
          am_hal_i2s_dma_error(pI2SHandle, AM_HAL_I2S_XFER_RX);
        }
        g_ui32I2SDmaCpl[1] = 1;
        g_ui32I2S0_RXCpl = 1;
    }

    if (ui32Status & AM_HAL_I2S_INT_TXDMACPL)
    {
        uint32_t ui32DMAStatus;
        am_hal_i2s_dma_status_get(pI2SHandle, &ui32DMAStatus, AM_HAL_I2S_XFER_TX);
        if ( ui32DMAStatus & AM_HAL_I2S_STAT_DMA_RX_ERR )
        {
          //
          // Clear DMAERR bit.
          //
          am_hal_i2s_dma_error(pI2SHandle, AM_HAL_I2S_XFER_TX);
        }
        g_ui32I2SDmaCpl[0] = 1;
        g_ui32I2S0_TXCpl = 1;
    }

    if (ui32Status & AM_HAL_I2S_INT_IPB)
    {
      am_hal_i2s_ipb_interrupt_service(pI2SHandle);
    }

//
// If master = I2S0, this is the I2S1 slave handler.
//
#else
    am_hal_i2s_interrupt_status_get(pI2SSlaveHandle, &ui32Status, true);
    am_hal_i2s_interrupt_clear(pI2SSlaveHandle, ui32Status);
    if (ui32Status & AM_HAL_I2S_INT_RXDMACPL)
    {
        uint32_t ui32DMAStatus;
        am_hal_i2s_dma_status_get(pI2SSlaveHandle, &ui32DMAStatus,             \
                                  AM_HAL_I2S_XFER_RX);
        //
        // The RX/TX DMACPL interrupt asserts when the programmed DMA completes,
        // or ends with an error condition.
        //
        if ( ui32DMAStatus & AM_HAL_I2S_STAT_DMA_RX_ERR )
        {
          //
          // Clear DMAERR bit.
          //
          am_hal_i2s_dma_error(pI2SSlaveHandle, AM_HAL_I2S_XFER_RX);
        }
       g_ui32I2SDmaCpl[3] = 1;
        g_ui32I2S1_RXCpl = 1;
    }

    if (ui32Status & AM_HAL_I2S_INT_TXDMACPL)
    {
        uint32_t ui32DMAStatus;
        am_hal_i2s_dma_status_get(pI2SSlaveHandle, &ui32DMAStatus,             \
                                  AM_HAL_I2S_XFER_TX);
        if ( ui32DMAStatus & AM_HAL_I2S_STAT_DMA_RX_ERR )
        {
          //
          // Clear DMAERR bit.
          //
          am_hal_i2s_dma_error(pI2SSlaveHandle, AM_HAL_I2S_XFER_TX);
        }
        g_ui32I2SDmaCpl[2] = 1;
        g_ui32I2S1_TXCpl = 1;
    }

    if (ui32Status & AM_HAL_I2S_INT_IPB)
    {
      am_hal_i2s_ipb_interrupt_service(pI2SSlaveHandle);
    }
#endif
}

//*****************************************************************************
//
// I2S initialization.
//
//*****************************************************************************
void
i2s_init(void)
{
    //
    // Configure the necessary pins.
    //
    am_hal_gpio_pincfg_t sPinCfg =
    {
      .GP.cfg_b.eGPOutCfg = 1,
      .GP.cfg_b.ePullup   = 0
    };

    sPinCfg.GP.cfg_b.uFuncSel = I2S_DATA_OUT_GPIO_FUNC;
    am_hal_gpio_pinconfig(I2S_DATA_OUT_GPIO_PIN, sPinCfg);
    sPinCfg.GP.cfg_b.uFuncSel = I2S_DATA_IN_GPIO_FUNC;
    am_hal_gpio_pinconfig(I2S_DATA_IN_GPIO_PIN, sPinCfg);

    sPinCfg.GP.cfg_b.uFuncSel = I2S_CLK_GPIO_FUNC;
    am_hal_gpio_pinconfig(I2S_CLK_GPIO_PIN, sPinCfg);
    sPinCfg.GP.cfg_b.uFuncSel = I2S_WS_GPIO_FUNC;
    am_hal_gpio_pinconfig(I2S_WS_GPIO_PIN, sPinCfg);

    am_hal_i2s_initialize(I2S_MODULE_MASTER, &pI2SHandle);
    am_hal_i2s_power_control(pI2SHandle, AM_HAL_I2S_POWER_ON, false);
    am_hal_i2s_configure(pI2SHandle, &g_sI2SConfig);
    am_hal_i2s_enable(pI2SHandle);

    sPinCfg.GP.cfg_b.uFuncSel = I2S_SLAVE_DATA_OUT_GPIO_FUNC;
    am_hal_gpio_pinconfig(I2S_SLAVE_DATA_OUT_GPIO_PIN, sPinCfg);
    sPinCfg.GP.cfg_b.uFuncSel = I2S_SLAVE_DATA_IN_GPIO_FUNC;
    am_hal_gpio_pinconfig(I2S_SLAVE_DATA_IN_GPIO_PIN, sPinCfg);

    sPinCfg.GP.cfg_b.uFuncSel = I2S_SLAVE_CLK_GPIO_FUNC;
    am_hal_gpio_pinconfig(I2S_SLAVE_CLK_GPIO_PIN, sPinCfg);
    sPinCfg.GP.cfg_b.uFuncSel = I2S_SLAVE_WS_GPIO_FUNC;
    am_hal_gpio_pinconfig(I2S_SLAVE_WS_GPIO_PIN, sPinCfg);

    am_hal_i2s_initialize(I2S_MODULE_SLAVE, &pI2SSlaveHandle);
    am_hal_i2s_power_control(pI2SSlaveHandle, AM_HAL_I2S_POWER_ON, false);
    am_hal_i2s_configure(pI2SSlaveHandle, &g_sI2SConfig_slave);
    am_hal_i2s_enable(pI2SSlaveHandle);

    //
    // Enable hfrc2/xths.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFRC2_START, false);
    am_util_delay_ms(200);

    //
    // Enable EXTCLK32M.
    //
    if ( (eAM_HAL_I2S_CLKSEL_XTHS_EXTREF_CLK <= g_sI2SConfig.eClock  &&       \
          g_sI2SConfig.eClock <= eAM_HAL_I2S_CLKSEL_XTHS_500KHz) ||           \
         (eAM_HAL_I2S_CLKSEL_XTHS_EXTREF_CLK <= g_sI2SConfig_slave.eClock  && \
          g_sI2SConfig_slave.eClock <= eAM_HAL_I2S_CLKSEL_XTHS_500KHz) )
    {
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_NORMAL, 0);
        am_util_delay_ms(200);
    }
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
    //
    // Initialize the printf interface for ITM output.
    //
    am_bsp_itm_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();

#if USE_I2S1_MASTER == 0
    am_util_stdio_printf("I2S Loopback Test: Master = I2S0, Slave = I2S1.\n\n");
#else
    am_util_stdio_printf("I2S Loopback Test: Master = I2S1, Slave = I2S0.\n\n");
#endif

    //
    // Initialize data.
    //
    for (int i = 0; i < BUFFER_SIZE_BYTES / 4; i++)
    {
        g_ui32I2STxDataBuffer[i] = (i & 0xFF) | 0xAB0000;
        g_ui32I2STxDataBuffer_slave[i] = (i & 0xFF) | 0xAB0000;
    }

    g_I2S0 = (I2S0_Type*)0x40208000;
    g_I2S1 = (I2S0_Type*)0x40209000;

    i2s_init();

    //
    // I2S DMA config.
    //
    am_hal_i2s_dma_configure(pI2SSlaveHandle, &g_sI2SConfig_slave,             \
                             &sTransfer_slave);
    NVIC_EnableIRQ(i2s_interrupts[I2S_MODULE_SLAVE]);
    am_hal_i2s_dma_configure(pI2SHandle, &g_sI2SConfig, &sTransfer);
    NVIC_EnableIRQ(i2s_interrupts[I2S_MODULE_MASTER]);
    am_hal_interrupt_master_enable();

    am_hal_i2s_dma_transfer_start(pI2SHandle, &g_sI2SConfig);
    am_hal_i2s_dma_transfer_start(pI2SSlaveHandle, &g_sI2SConfig_slave);

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        if ( (g_ui32I2S0_RXCpl == g_ui32I2SDmaCpl[1]) &&                      \
             (g_ui32I2S0_TXCpl == g_ui32I2SDmaCpl[0]) &&                      \
             (g_ui32I2S1_RXCpl == g_ui32I2SDmaCpl[3]) &&                      \
             (g_ui32I2S1_TXCpl == g_ui32I2SDmaCpl[2]))
        {
            am_hal_i2s_transfer_complete(pI2SHandle);
            am_hal_i2s_transfer_complete(pI2SSlaveHandle);
            am_hal_i2s_disable(pI2SHandle);
            am_hal_i2s_disable(pI2SSlaveHandle);

            g_ui32I2SDmaCpl[0] = g_ui32I2SDmaCpl[1] = g_ui32I2SDmaCpl[2] =    \
                                 g_ui32I2SDmaCpl[3] = 0;

            if ( check_i2s_data(BUFFER_SIZE_BYTES / 4,                        \
                 g_ui32I2SRxDataBuffer_slave, g_ui32I2STxDataBuffer)          \
                 && check_i2s_data(BUFFER_SIZE_BYTES / 4,                     \
                 g_ui32I2SRxDataBuffer, g_ui32I2STxDataBuffer_slave) )
            {
                g_ui32I2SDmaCpl[4] = 1;
                am_util_stdio_printf("I2S Loopback Test PASSED!\n");
            }
            else
            {
                am_util_stdio_printf("I2S Loopback Test FAILED!\n");
            }

            while(1);
        }

        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
