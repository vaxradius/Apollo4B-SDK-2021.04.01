//*****************************************************************************
//
//! @file pdm_to_i2s.c
//!
//! @brief An example to show PDM to I2S(slave) operation.
//!
//! Purpose: This example enables the PDM and I2S interface to collect audio signals from
//!          an external microphone, I2S module using pingpong buffer to interact with PDM,
//!          and start transaction when mclk is supplied(from external I2S master).
//!          Notice: external mclk should be supplied first at this example.
//! The required pin connections are:
//!
//! Printing takes place over the ITM at 1M Baud.
//!
//! GPIO 50 - PDM0 CLK
//! GPIO 51 - PDM0 DATA
//!
//! GPIO 52 - PDM1 CLK
//! GPIO 53 - PDM1 DATA
//!
//! GPIO 54 - PDM2 CLK
//! GPIO 55 - PDM2 DATA
//!
//! GPIO 56 - PDM3 CLK
//! GPIO 57 - PDM3 DATA
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
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "SEGGER_RTT.h"

//*****************************************************************************
//
// Insert compiler version at compile time.
//
//*****************************************************************************


//*****************************************************************************
//
// Example parameters.
//
//*****************************************************************************
#define  FFT_PDM_MODULE             0

//*****************************************************************************
//
// PDM pins
//
//*****************************************************************************
#if FFT_PDM_MODULE == 0
#define PDM_DATA_GPIO_FUNC  AM_HAL_PIN_51_PDM0_DATA
#define PDM_DATA_GPIO_PIN   51
#define PDM_CLK_GPIO_FUNC   AM_HAL_PIN_50_PDM0_CLK
#define PDM_CLK_GPIO_PIN    50
#elif FFT_PDM_MODULE == 1
#define PDM_DATA_GPIO_FUNC  AM_HAL_PIN_53_PDM1_DATA
#define PDM_DATA_GPIO_PIN   53
#define PDM_CLK_GPIO_FUNC   AM_HAL_PIN_52_PDM1_CLK
#define PDM_CLK_GPIO_PIN    52
#elif FFT_PDM_MODULE == 2
#define PDM_DATA_GPIO_FUNC  AM_HAL_PIN_55_PDM2_DATA
#define PDM_DATA_GPIO_PIN   55
#define PDM_CLK_GPIO_FUNC   AM_HAL_PIN_54_PDM2_CLK
#define PDM_CLK_GPIO_PIN    54
#elif FFT_PDM_MODULE == 3
#define PDM_DATA_GPIO_FUNC  AM_HAL_PIN_57_PDM3_DATA
#define PDM_DATA_GPIO_PIN   57
#define PDM_CLK_GPIO_FUNC   AM_HAL_PIN_56_PDM3_CLK
#define PDM_CLK_GPIO_PIN    56
#endif

#define PDM_FFT_SIZE                320 //20ms
#define PDM_FFT_BYTES               (PDM_FFT_SIZE * 4)
#define PRINT_PDM_DATA              0
#define PRINT_FFT_DATA              0


//! PDM interrupts.
static const IRQn_Type pdm_interrupts[] =
{
    PDM0_IRQn,
    PDM1_IRQn,
    PDM2_IRQn,
    PDM3_IRQn
};

//
// Take over the interrupt handler for whichever PDM we're using.
//
#define example_pdm_isr     am_pdm_isr1(FFT_PDM_MODULE)
#define am_pdm_isr1(n)      am_pdm_isr(n)
#define am_pdm_isr(n)       am_pdm ## n ## _isr

#define FIFO_THRESHOLD_CNT      16
#define DMA_BYTES               PDM_FFT_BYTES

//*****************************************************************************
//
// PDM configuration information.
//
//*****************************************************************************
void *PDMHandle;

am_hal_pdm_config_t g_sPdmConfig =
{

#ifdef AM_PART_APOLLO4B
    //
    // Example setting:
    //  1.5MHz PDM CLK OUT:
    //      AM_HAL_PDM_CLK_HFRC2ADJ_24_576MHZ, AM_HAL_PDM_MCLKDIV_1, AM_HAL_PDM_PDMA_CLKO_DIV7
    //  16.00KHz 24bit Sampling:
    //      DecimationRate = 48
    //
    //.ePDMClkSpeed = AM_HAL_PDM_CLK_HFRC2ADJ_24_576MHZ,
    .ePDMClkSpeed = AM_HAL_PDM_CLK_HFRC_24MHZ,
    .eClkDivider = AM_HAL_PDM_MCLKDIV_1,
    .ePDMAClkOutDivder = AM_HAL_PDM_PDMA_CLKO_DIV7,
    .ui32DecimationRate = 48,

#else
    //
    // Example setting:
    //  1.5MHz PDM CLK OUT:
    //      AM_HAL_PDM_CLK_24MHZ, AM_HAL_PDM_MCLKDIV_1, AM_HAL_PDM_PDMA_CLKO_DIV7
    //  15.625KHz 24bit Sampling:
    //      DecimationRate = 48
    //
    .ePDMClkSpeed = AM_HAL_PDM_CLK_24MHZ,
    .eClkDivider = AM_HAL_PDM_MCLKDIV_1,
    .ePDMAClkOutDivder = AM_HAL_PDM_PDMA_CLKO_DIV7,
    .ui32DecimationRate = 48,
#endif

    .eLeftGain = AM_HAL_PDM_GAIN_P105DB,
    .eRightGain = AM_HAL_PDM_GAIN_P105DB,
    .eStepSize = AM_HAL_PDM_GAIN_STEP_0_13DB,

    .bHighPassEnable = AM_HAL_PDM_HIGH_PASS_ENABLE,
    .ui32HighPassCutoff = 0x3,
    .bDataPacking = 1,
    .ePCMChannels = AM_HAL_PDM_CHANNEL_STEREO,

    .bPDMSampleDelay = AM_HAL_PDM_CLKOUT_PHSDLY_NONE,
    .ui32GainChangeDelay = AM_HAL_PDM_CLKOUT_DELAY_NONE,

    .bSoftMute = 0,
    .bLRSwap = 0,
};

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
uint32_t g_ui32FifoOVFCount = 0;
volatile bool g_bPDMDataReady = false;
uint32_t g_ui32SampleFreq;

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//*****************************************************************************
//
// I2S
//
//*****************************************************************************
#define     I2S_MODULE          0   // I2S0
#define     USE_DMA             1

#if USE_DMA
#define BUFFER_SIZE_BYTES               PDM_FFT_BYTES  //(8*1024)

//! RX size = TX size * output sample rate /internal sample rate
//! Notice .eClock from g_sI2SConfig and g_sI2SConfig_slave.
#define BUFFER_SIZE_ASRC_RX_BYTES       PDM_FFT_BYTES

#endif

//! I2S interrupts.
static const IRQn_Type i2s_interrupts[] =
{
    I2S0_IRQn,
    I2S1_IRQn
};

#if I2S_MODULE == 0
#define I2S_DATA_IN_GPIO_FUNC  AM_HAL_PIN_4_I2S0_SDIN
#define I2S_DATA_IN_GPIO_PIN   4
#define I2S_DATA_OUT_GPIO_FUNC  AM_HAL_PIN_6_I2S0_SDOUT
#define I2S_DATA_OUT_GPIO_PIN   6
#define I2S_CLK_GPIO_FUNC   AM_HAL_PIN_5_I2S0_CLK
#define I2S_CLK_GPIO_PIN    5
#define I2S_WS_GPIO_FUNC    AM_HAL_PIN_7_I2S0_WS
#define I2S_WS_GPIO_PIN     7
#endif

void *I2SHandle;

// Programmer Reference setting.
static am_hal_i2s_io_signal_t g_sI2SIOConfig =
{
  .eFyncCpol = AM_HAL_I2S_IO_FSYNC_CPOL_HIGH,
  .eTxCpol = AM_HAL_I2S_IO_TX_CPOL_FALLING,
  .eRxCpol = AM_HAL_I2S_IO_RX_CPOL_RISING,
};


static am_hal_i2s_data_format_t g_sI2SDataConfig =
{
  .ePhase = AM_HAL_I2S_DATA_PHASE_SINGLE,
  //.eDataDelay = 0x1,
  .eDataJust = AM_HAL_I2S_DATA_JUSTIFIED_LEFT,

  .eChannelLenPhase1 = AM_HAL_I2S_FRAME_32BITS_WDLEN, //32bits
  .eChannelLenPhase2 = AM_HAL_I2S_FRAME_32BITS_WDLEN, //32bits
  .eSampleLenPhase1 = AM_HAL_I2S_FRAME_24BITS_WDLEN,
  .eSampleLenPhase2 = AM_HAL_I2S_FRAME_24BITS_WDLEN
};

//*****************************************************************************
//
// I2S configuration information.
//
//*****************************************************************************
static am_hal_i2s_config_t g_sI2SConfig =
{
    .eClock               = eAM_HAL_I2S_CLKSEL_HFRC_3MHz,
    .eDiv3                = 1,
    .eMode                = AM_HAL_I2S_IO_MODE_MASTER,
    .eXfer                = AM_HAL_I2S_XFER_RXTX,
    .ui32ChnNumber        = 2,
    .eData                = &g_sI2SDataConfig,
    .eIO                  = &g_sI2SIOConfig
};

// Transfer setting.
static am_hal_i2s_transfer_t sTransfer =
{
  .ui32RxTotalCount = 0, //!!! Must less than 16K bytes.
  .ui32RxTargetAddr = 0,

  .ui32TxTotalCount = 0,
  .ui32TxTargetAddr = 0

};

//
// pingpong machine

#define PING_PANG 1

#define PP_PINGPANG(v) \
    (v == PP_PING? PP_PANG: PP_PING)


#define AUDIO_BUFFER_COUNT (4)

//pingpang buffer size: PMD_DMATOTCOUNT(x2)
AM_SHARED_RW uint32_t g_ui32PingPangBuffer[PDM_FFT_SIZE + PDM_FFT_SIZE];


typedef enum
{
    PP_PING = 0,
    PP_PANG = 1
} AF_PP_T;

typedef enum
{
    AUD_STREAM_PING = 0,
    AUD_STREAM_PANG,
    AUD_STREAM_NUM,
} AUD_STREAM_T;

typedef struct
{
    //Pingpang index
    AF_PP_T pp_index;

    uint8_t *dma_buf_ptr;
    uint32_t dma_buf_size;
} pp_stream_cfg_t;

pp_stream_cfg_t pp_stream[AUD_STREAM_NUM];
volatile uint8_t readIndex = PP_PANG;

//*****************************************************************************
//
// PINGPANG initialization.
//
//*****************************************************************************
void
ping_pang_init(void)
{
   pp_stream[AUD_STREAM_PING].dma_buf_ptr = (uint8_t *)g_ui32PingPangBuffer;
   pp_stream[AUD_STREAM_PING].dma_buf_size = PDM_FFT_BYTES;
   pp_stream[AUD_STREAM_PING].pp_index = PP_PING;

   pp_stream[AUD_STREAM_PANG].dma_buf_ptr = (uint8_t *)(&g_ui32PingPangBuffer[PDM_FFT_SIZE]);
   pp_stream[AUD_STREAM_PANG].dma_buf_size = PDM_FFT_BYTES;
   pp_stream[AUD_STREAM_PANG].pp_index = PP_PANG;

}

//*****************************************************************************
//
// PDM initialization.
//
//*****************************************************************************
void
pdm_init(void)
{
    //
    // Initialize, power-up, and configure the PDM.
    //
    am_hal_pdm_initialize(FFT_PDM_MODULE, &PDMHandle);
    am_hal_pdm_power_control(PDMHandle, AM_HAL_PDM_POWER_ON, false);

#ifdef AM_PART_APOLLO4B
    // use external XTHS, not reference clock
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, false);

    // Enable HFRC2
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFRC2_START, false);
    am_util_delay_us(200);      // wait for FLL to lock

    // set HF2ADJ for 24.576MHz output
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HF2ADJ_ENABLE, false);
    am_util_delay_us(500);      // wait for adj to apply
#endif

    am_hal_pdm_configure(PDMHandle, &g_sPdmConfig);

    //
    // Configure the necessary pins.
    //
    am_hal_gpio_pincfg_t sPinCfg = {0};

    sPinCfg.GP.cfg_b.uFuncSel = PDM_DATA_GPIO_FUNC;
    am_hal_gpio_pinconfig(PDM_DATA_GPIO_PIN, sPinCfg);
    sPinCfg.GP.cfg_b.uFuncSel = PDM_CLK_GPIO_FUNC;
    am_hal_gpio_pinconfig(PDM_CLK_GPIO_PIN, sPinCfg);

    am_hal_pdm_fifo_threshold_setup(PDMHandle, FIFO_THRESHOLD_CNT);

    //
    // Configure and enable PDM interrupts (set up to trigger on DMA
    // completion).
    //
    am_hal_pdm_interrupt_enable(PDMHandle, (AM_HAL_PDM_INT_DERR
                                            | AM_HAL_PDM_INT_DCMP
                                            | AM_HAL_PDM_INT_UNDFL
                                            | AM_HAL_PDM_INT_OVF));

    NVIC_SetPriority(pdm_interrupts[FFT_PDM_MODULE], AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(pdm_interrupts[FFT_PDM_MODULE]);

}

//*****************************************************************************
//
// Print PDM configuration data.
//
//*****************************************************************************
void
pdm_config_print(void)
{
    uint32_t ui32PDMClk;
    uint32_t ui32MClkDiv;
    uint32_t ui32DivClkQ;
    float fFrequencyUnits;

    //
    // Read the config structure to figure out what our internal clock is set
    // to.
    //
    switch (g_sPdmConfig.eClkDivider)
    {
        case AM_HAL_PDM_MCLKDIV_3: ui32MClkDiv = 3; break;
        case AM_HAL_PDM_MCLKDIV_2: ui32MClkDiv = 2; break;
        case AM_HAL_PDM_MCLKDIV_1: ui32MClkDiv = 1; break;

        default:
            ui32MClkDiv = 1;
    }

    switch (g_sPdmConfig.ePDMAClkOutDivder)
    {
        case AM_HAL_PDM_PDMA_CLKO_DIV15: ui32DivClkQ = 15; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV14: ui32DivClkQ = 14; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV13: ui32DivClkQ = 13; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV12: ui32DivClkQ = 12; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV11: ui32DivClkQ = 11; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV10: ui32DivClkQ = 10; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV9: ui32DivClkQ = 9; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV8: ui32DivClkQ = 8; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV7: ui32DivClkQ = 7; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV6: ui32DivClkQ = 6; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV5: ui32DivClkQ = 5; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV4: ui32DivClkQ = 4; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV3: ui32DivClkQ = 3; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV2: ui32DivClkQ = 2; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV1: ui32DivClkQ = 1; break;
        default:
            ui32DivClkQ = 1;
    }

#ifdef AM_PART_APOLLO4B
    switch (g_sPdmConfig.ePDMClkSpeed)
    {
        case AM_HAL_PDM_CLK_HFRC2ADJ_24_576MHZ:     ui32PDMClk = 24576000; break;
        case AM_HAL_PDM_CLK_HFXTAL:                 ui32PDMClk = 32000000; break;
        case AM_HAL_PDM_CLK_HFRC_24MHZ:             ui32PDMClk = 24000000; break;

        default:
            ui32PDMClk = 24000000;
    }
#else
    switch (g_sPdmConfig.ePDMClkSpeed)
    {
        case AM_HAL_PDM_CLK_48MHZ:      ui32PDMClk = 48000000; break;
        case AM_HAL_PDM_CLK_24MHZ:      ui32PDMClk = 24000000; break;
        case AM_HAL_PDM_CLK_12MHZ:      ui32PDMClk = 12000000; break;
        case AM_HAL_PDM_CLK_6MHZ:       ui32PDMClk =  6000000; break;
        case AM_HAL_PDM_CLK_3MHZ:       ui32PDMClk =  3000000; break;
        case AM_HAL_PDM_CLK_1_5MHZ:     ui32PDMClk =  1500000; break;
        case AM_HAL_PDM_CLK_750KHZ:     ui32PDMClk =   750000; break;
        case AM_HAL_PDM_CLK_HS_CRYSTAL: ui32PDMClk =   0; break;

        default:
            ui32PDMClk = 24000000;
    }
#endif
    //
    // Record the effective sample frequency. We'll need it later to print the
    // loudest frequency from the sample.
    //
    g_ui32SampleFreq = (ui32PDMClk /
                        ((ui32MClkDiv + 1) * (ui32DivClkQ + 1) * 2 * g_sPdmConfig.ui32DecimationRate));

    ui32PDMClk =  (ui32PDMClk /
                        ((ui32MClkDiv + 1) * (ui32DivClkQ + 1)));   // actual PDM CLK output

    fFrequencyUnits = (float) g_ui32SampleFreq / (float) PDM_FFT_SIZE;

    am_util_stdio_printf("PDM Settings:\n");
    am_util_stdio_printf("PDM Clock (Hz):         %12d\n", ui32PDMClk);
    am_util_stdio_printf("Decimation Rate:        %12d\n", g_sPdmConfig.ui32DecimationRate);
    am_util_stdio_printf("Effective Sample Freq.: %12d\n", g_ui32SampleFreq);
    am_util_stdio_printf("FFT Length:             %12d\n\n", PDM_FFT_SIZE);
    am_util_stdio_printf("FFT Resolution: %15.3f Hz\n", fFrequencyUnits);
}

//*****************************************************************************
//
// Start a transaction to get some number of bytes from the PDM interface.
//
//*****************************************************************************
void
pdm_data_get(void)
{
    //
    // Configure DMA and target address.
    //
    am_hal_pdm_transfer_t sTransfer;

#if PING_PANG

    readIndex = PP_PINGPANG(readIndex);

    sTransfer.ui32TargetAddr = (uint32_t ) pp_stream[readIndex].dma_buf_ptr;
    sTransfer.ui32TotalCount = pp_stream[readIndex].dma_buf_size;

#endif
    //
    // Start the data transfer.
    //

    am_hal_pdm_dma_start(PDMHandle, &sTransfer);

}

//*****************************************************************************
//
// PDM interrupt handler.
//
//*****************************************************************************
void
example_pdm_isr(void)
{
    uint32_t ui32Status;

    //
    // Read the interrupt status.
    //
    am_hal_pdm_interrupt_status_get(PDMHandle, &ui32Status, true);
    am_hal_pdm_interrupt_clear(PDMHandle, ui32Status);
    //
    // Once our DMA transaction completes, we will disable the PDM and send a
    // flag back down to the main routine. Disabling the PDM is only necessary
    // because this example only implemented a single buffer for storing FFT
    // data. More complex programs could use a system of multiple buffers to
    // allow the CPU to run the FFT in one buffer while the DMA pulls PCM data
    // into another buffer.
    //
    if (ui32Status & AM_HAL_PDM_INT_DCMP)
    {

        //am_util_stdio_printf("pdm isr.\n");
        g_bPDMDataReady = true;

        pdm_data_get();
    }

     if (ui32Status & AM_HAL_PDM_INT_OVF)
     {
        uint32_t count = am_hal_pdm_fifo_count_get(PDMHandle);
        am_hal_pdm_fifo_flush(PDMHandle);

        g_ui32FifoOVFCount++;
     }
}

//*****************************************************************************
// i2s_init
//*****************************************************************************
void
i2s_init(void)
{
    //
    // When changing the I2S clock source, no matter whether HFRC2 is being used or not,
    // HFRC2 has to be enabled first to make clock source changing effective.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFRC2_START, false);
    am_util_delay_ms(200);

    if ( (eAM_HAL_I2S_CLKSEL_XTHS_EXTREF_CLK <= g_sI2SConfig.eClock  && g_sI2SConfig.eClock <= eAM_HAL_I2S_CLKSEL_XTHS_500KHz) ) //enable EXTCLK32M
    {
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_NORMAL, 0);
        am_util_delay_ms(200);
    }

    //
    // Configure the necessary pins.
    //
    am_hal_gpio_pincfg_t sPinCfg =
    {
      .GP.cfg_b.eGPOutCfg = 1,
      .GP.cfg_b.ePullup   = 0
    };

    sPinCfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_12_I2S0_SDOUT;
    am_hal_gpio_pinconfig(12, sPinCfg);
	
    sPinCfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_11_I2S0_CLK;
    am_hal_gpio_pinconfig(11, sPinCfg);
	
    sPinCfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_13_I2S0_WS;
    am_hal_gpio_pinconfig(13, sPinCfg);

    am_hal_i2s_initialize(I2S_MODULE, &I2SHandle);
    am_hal_i2s_power_control(I2SHandle, AM_HAL_I2S_POWER_ON, false);
    am_hal_i2s_configure(I2SHandle, &g_sI2SConfig);
    am_hal_i2s_enable(I2SHandle);
}

//*****************************************************************************
// am_hal_i2s0_interrupt_service
//*****************************************************************************
static void am_hal_i2s0_interrupt_service(void)
{
    uint8_t reverseIndex = PP_PINGPANG(readIndex);
    sTransfer.ui32TxTargetAddr = (uint32_t)pp_stream[reverseIndex].dma_buf_ptr;

    am_hal_i2s_dma_transfer_continue(I2SHandle, &g_sI2SConfig, &sTransfer);
}

void
am_dspi2s0_isr()
{
    uint32_t ui32Status;

    am_hal_i2s_interrupt_status_get(I2SHandle, &ui32Status, true);
    am_hal_i2s_interrupt_clear(I2SHandle, ui32Status);


    if (ui32Status & AM_HAL_I2S_INT_RXDMACPL)
    {
        uint32_t ui32DMAStatus;
        am_hal_i2s_dma_status_get(I2SHandle, &ui32DMAStatus, AM_HAL_I2S_XFER_RX);

        //
        // The RX/TX DMACPL interrupt asserts when the programmed DMA completes,
        // or end with an errorcondition.
        //
        if ( ui32DMAStatus & AM_HAL_I2S_STAT_DMA_RX_ERR )
        {
            // cleare DMAERR bit
            am_hal_i2s_dma_error(I2SHandle, AM_HAL_I2S_XFER_RX);
        }

        //am_util_stdio_printf("i2s0 rxcmpt.\n");
    }

    if (ui32Status & AM_HAL_I2S_INT_TXDMACPL)
    {
        uint32_t ui32DMAStatus;
        am_hal_i2s_dma_status_get(I2SHandle, &ui32DMAStatus, AM_HAL_I2S_XFER_TX);

        if ( ui32DMAStatus & AM_HAL_I2S_STAT_DMA_TX_ERR )
        {
            // cleare DMAERR bit
            am_hal_i2s_dma_error(I2SHandle, AM_HAL_I2S_XFER_TX);
        }

        //am_util_stdio_printf("i2s0 txcmpt.\n");

        am_hal_i2s0_interrupt_service();
    }


    //
    if (ui32Status & AM_HAL_I2S_INT_IPB)
    {
      am_hal_i2s_ipb_interrupt_service(I2SHandle);
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

    am_bsp_low_power_init();

    ping_pang_init();


    //
    // Initialize PDM-to-PCM module
    //
    pdm_init();
    am_hal_pdm_enable(PDMHandle);

    //
    // Initialize I2S.
    //
    i2s_init();
    NVIC_SetPriority(i2s_interrupts[I2S_MODULE], AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(i2s_interrupts[I2S_MODULE]);
    am_hal_interrupt_master_enable();

    //
    //
    // Start data conversion
    //
    pdm_data_get();


    //Start I2S data transaction.
#if PING_PANG
    sTransfer.ui32TxTargetAddr = (uint32_t)pp_stream[PP_PING].dma_buf_ptr;
    sTransfer.ui32TxTotalCount = pp_stream[PP_PING].dma_buf_size / 4;   //32bits/sample
#endif
    am_hal_i2s_dma_configure(I2SHandle, &g_sI2SConfig, &sTransfer);
    am_hal_i2s_dma_transfer_start(I2SHandle, &g_sI2SConfig);


    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}


void pdm_deinitialization(void *pHandle)
{
    am_hal_pdm_interrupt_clear(pHandle, (AM_HAL_PDM_INT_DERR
                                            | AM_HAL_PDM_INT_DCMP
                                            | AM_HAL_PDM_INT_UNDFL
                                            | AM_HAL_PDM_INT_OVF));

    am_hal_pdm_interrupt_disable(pHandle, (AM_HAL_PDM_INT_DERR
                                            | AM_HAL_PDM_INT_DCMP
                                            | AM_HAL_PDM_INT_UNDFL
                                            | AM_HAL_PDM_INT_OVF));

    NVIC_DisableIRQ(pdm_interrupts[FFT_PDM_MODULE]);

    am_hal_gpio_pinconfig(PDM_CLK_GPIO_PIN, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(PDM_DATA_GPIO_PIN, am_hal_gpio_pincfg_disabled);


    am_hal_pdm_disable(pHandle);
    am_hal_pdm_power_control(pHandle, AM_HAL_PDM_POWER_OFF, false);

    am_hal_pdm_deinitialize(pHandle);
}


void i2s_deinitialization(void *pHandle)
{
    am_hal_i2s_transfer_complete(pHandle);

    am_hal_i2s_interrupt_clear(pHandle, (AM_HAL_I2S_INT_RXDMACPL | AM_HAL_I2S_INT_TXDMACPL));
    am_hal_i2s_interrupt_disable(pHandle, (AM_HAL_I2S_INT_RXDMACPL | AM_HAL_I2S_INT_TXDMACPL));

    NVIC_DisableIRQ(i2s_interrupts[I2S_MODULE]);

    am_hal_gpio_pinconfig(I2S_WS_GPIO_PIN, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(I2S_CLK_GPIO_PIN, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(I2S_DATA_OUT_GPIO_PIN, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(I2S_DATA_IN_GPIO_PIN, am_hal_gpio_pincfg_disabled);

    am_hal_i2s_disable(pHandle);
    am_hal_i2s_power_control(pHandle, AM_HAL_I2S_POWER_OFF, false);

    am_hal_i2s_deinitialize(pHandle);
}
