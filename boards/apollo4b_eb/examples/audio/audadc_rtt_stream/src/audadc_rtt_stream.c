//*****************************************************************************
//
//! @file audadc_lpmode0_dma.c
//!
//! @brief This example takes samples with the AUDADC at high-speed using DMA.
//!
//! Purpose: This example shows the CTIMER-A3 triggering repeated samples of an external
//! input at 1.2Msps in LPMODE0.  The example uses the CTIMER-A3 to trigger
//! AUDADC sampling.  Each data point is 128 sample average and is transferred
//! from the AUDADC FIFO into an SRAM buffer using DMA.
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "SEGGER_RTT.h"

//*****************************************************************************
//
// Insert compiler version at compile time.
//
//*****************************************************************************
#define STRINGIZE_VAL(n)                    STRINGIZE_VAL2(n)
#define STRINGIZE_VAL2(n)                   #n

#ifdef __GNUC__
#define COMPILER_VERSION                    ("GCC " __VERSION__)
#elif defined(__ARMCC_VERSION)
#define COMPILER_VERSION                    ("ARMCC " STRINGIZE_VAL(__ARMCC_VERSION))
#elif defined(__KEIL__)
#define COMPILER_VERSION                    "KEIL_CARM " STRINGIZE_VAL(__CA__)
#elif defined(__IAR_SYSTEMS_ICC__)
#define COMPILER_VERSION                    __VERSION__
#else
#define COMPILER_VERSION                    "Compiler unknown"
#endif

AUDADC_Type* g_adc;
MCUCTRL_Type* g_mcuctrl;
//*****************************************************************************
//
// Define a circular buffer to hold the AUDADC samples
//
//*****************************************************************************

//
// AUDADC Sample buffer.
//
#define AUDADC_SAMPLE_BUF_SIZE      (16*1024)
AM_SHARED_RW uint32_t g_ui32AUDADCSampleBuffer[AUDADC_SAMPLE_BUF_SIZE];

#define AUDADC_DATA_BUFFER_SIZE     (2*AUDADC_SAMPLE_BUF_SIZE)
AM_SHARED_RW int16_t g_in16AudioDataBuffer[AUDADC_DATA_BUFFER_SIZE];

//
// RTT streaming buffer
//
#define TIMEOUT                     400000      // RTT streaming timeout loop count
#define RTT_BUFFER_LENGTH           (256*1024)
#define AUDIO_SAMPLE_TO_RTT         (256*1024)
uint8_t g_rttRecorderBuffer[RTT_BUFFER_LENGTH];
AM_SHARED_RW int16_t g_in16SampleToRTT[AUDIO_SAMPLE_TO_RTT];
uint32_t g_ui32SampleToRTT = 0;

//
// AUDADC Device Handle.
//
static void *g_AUDADCHandle;

//
// AUDADC DMA complete flag.
//
volatile bool g_bAUDADCDMAComplete;

//
// AUDADC DMA error flag.
//
volatile bool g_bAUDADCDMAError;

volatile uint32_t g_ui32AdcFifoCount = 0;

//*****************************************************************************
//
// AUDADC gain configuration information.
//
//*****************************************************************************
am_hal_audadc_gain_config_t g_sAudadcGainConfig =
{
    .ui32LGA        = 33,       // 10.5 dB
    .ui32HGADELTA   = 2,        // delta from the LGA field
    .ui32LGB        = 33,       // 10.5 dB
    .ui32HGBDELTA   = 2,        // delta from the LGB field

    .eUpdateMode    = AM_HAL_AUDADC_GAIN_UPDATE_IMME,
};

//
// helper function.
//
void pcm_rtt_record_slow(void* pBuffer, uint32_t NumBytes)
{

    uint32_t timeout = TIMEOUT;
    uint32_t bytesEveryTime = 512;

    uint32_t bytes_stored;

    int32_t u32Bytes = NumBytes;
    void* data = pBuffer;

    for (; u32Bytes >= 0; u32Bytes -= bytesEveryTime )
    {
        while(timeout--);
        timeout = TIMEOUT;

        bytes_stored = SEGGER_RTT_Write(1, data, bytesEveryTime);
        while((bytes_stored != bytesEveryTime));

        data = (void*) ((uint32_t)data + bytesEveryTime);
    }
}

//*****************************************************************************
//
// Set up the core for sleeping, and then go to sleep.
//
//*****************************************************************************
void
sleep(void)
{
    //
    // Go to Deep Sleep.
    //
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

}

//*****************************************************************************
//
// Configure the AUDADC.
//
//*****************************************************************************
void
audadc_config_dma(void)
{
    am_hal_audadc_dma_config_t       AUDADCDMAConfig;

    //
    // Configure the AUDADC to use DMA for the sample transfer.
    //
    AUDADCDMAConfig.bDynamicPriority = true;
    AUDADCDMAConfig.ePriority = AM_HAL_AUDADC_PRIOR_SERVICE_IMMED;
    AUDADCDMAConfig.bDMAEnable = true;
    AUDADCDMAConfig.ui32SampleCount = AUDADC_SAMPLE_BUF_SIZE;
    AUDADCDMAConfig.ui32TargetAddress = (uint32_t)g_ui32AUDADCSampleBuffer;
    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_configure_dma(g_AUDADCHandle, &AUDADCDMAConfig))
    {
        am_util_stdio_printf("Error - configuring AUDADC DMA failed.\n");
    }

    //
    // Reset the AUDADC DMA flags.
    //
    g_bAUDADCDMAComplete = false;
    g_bAUDADCDMAError = false;
}

//*****************************************************************************
//
// Configure the AUDADC SLOT.
//
//*****************************************************************************
void
audadc_slot_config(void)
{
    am_hal_audadc_slot_config_t      AUDADCSlotConfig;

    //
    // Set up an AUDADC slot
    //
    AUDADCSlotConfig.eMeasToAvg      = AM_HAL_AUDADC_SLOT_AVG_1;
    AUDADCSlotConfig.ePrecisionMode  = AM_HAL_AUDADC_SLOT_12BIT;
    AUDADCSlotConfig.ui32TrkCyc      = 24;
    AUDADCSlotConfig.eChannel        = AM_HAL_AUDADC_SLOT_CHSEL_SE0;
    AUDADCSlotConfig.bWindowCompare  = true;
    AUDADCSlotConfig.bEnabled        = true;
    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_configure_slot(g_AUDADCHandle, 0, &AUDADCSlotConfig))
    {
        am_util_stdio_printf("Error - configuring AUDADC Slot 0 failed.\n");
    }

    AUDADCSlotConfig.eChannel        = AM_HAL_AUDADC_SLOT_CHSEL_SE1;
    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_configure_slot(g_AUDADCHandle, 1, &AUDADCSlotConfig))
    {
        am_util_stdio_printf("Error - configuring AUDADC Slot 1 failed.\n");
    }

}

//*****************************************************************************
//
// Configure the AUDADC.
//
//*****************************************************************************
void
audadc_config(void)
{
    //
    // Set up the AUDADC configuration parameters. These settings are reasonable
    // for accurate measurements at a low sample rate.
    //
    am_hal_audadc_config_t           AUDADCConfig =
    {
        .eClock             = AM_HAL_AUDADC_CLKSEL_HFRC2_48MHz, //AM_HAL_AUDADC_CLKSEL_XTHS_24MHz;
        .ePolarity          = AM_HAL_AUDADC_TRIGPOL_RISING,
        .eTrigger           = AM_HAL_AUDADC_TRIGSEL_SOFTWARE,
        .eClockMode         = AM_HAL_AUDADC_CLKMODE_LOW_LATENCY,
        .ePowerMode         = AM_HAL_AUDADC_LPMODE1,
        .eRepeat            = AM_HAL_AUDADC_REPEATING_SCAN,
        .eRepeatTrigger     = AM_HAL_AUDADC_RPTTRIGSEL_INT,     //AM_HAL_AUDADC_RPTTRIGSEL_INT;
        .eSampMode          = AM_HAL_AUDADC_SAMPMODE_MED,       //AM_HAL_AUDADC_SAMPMODE_LP,
    };

    //
    // Set up internal repeat trigger timer
    //
    am_hal_audadc_irtt_config_t      AUDADCIrttConfig =
    {
        .bIrttEnable        = true,
        .eClkDiv            =  AM_HAL_AUDADC_RPTT_CLK_DIV32,
        .ui32IrttCountMax   = 61,
    };

    //
    // Initialize the AUDADC and get the handle.
    //
    if ( AM_HAL_STATUS_SUCCESS != am_hal_audadc_initialize(0, &g_AUDADCHandle) )
    {
        am_util_stdio_printf("Error - reservation of the AUDADC instance failed.\n");
    }
    //
    // Power on the AUDADC.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_power_control(g_AUDADCHandle,
                                                          AM_HAL_SYSCTRL_WAKE,
                                                          false) )
    {
        am_util_stdio_printf("Error - AUDADC power on failed.\n");
    }

    //Enable hfrc2.
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFRC2_START, false);
    am_util_delay_us(200);
#if 0
    // hfrc2 adj.
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, false);

    // set HF2ADJ for 24.576MHz output
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HF2ADJ_ENABLE,false);

    am_util_delay_us(500);      // wait for adj to apply
#endif

    if ( AUDADCConfig.eClock == AM_HAL_AUDADC_CLKSEL_XTHS_24MHz )
    {
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_NORMAL, 0);
        am_util_delay_ms(200);
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_configure(g_AUDADCHandle, &AUDADCConfig))
    {
        am_util_stdio_printf("Error - configuring AUDADC failed.\n");
    }

    //
    // Set up internal repeat trigger timer
    //
    am_hal_audadc_configure_irtt(g_AUDADCHandle, &AUDADCIrttConfig);

    //
    // Enable the AUDADC.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_enable(g_AUDADCHandle))
    {
        am_util_stdio_printf("Error - enabling AUDADC failed.\n");
    }

    //
    // Enable internal repeat trigger timer
    //
    am_hal_audadc_irtt_enable(g_AUDADCHandle);


    //
    // Configure the AUDADC to use DMA for the sample transfer.
    //
    audadc_config_dma();

    //
    // For this example, the samples will be coming in slowly. This means we
    // can afford to wake up for every conversion.
    //
    am_hal_audadc_interrupt_enable(g_AUDADCHandle, AM_HAL_AUDADC_INT_FIFOOVR1 | AM_HAL_AUDADC_INT_FIFOOVR2 | AM_HAL_AUDADC_INT_DERR | AM_HAL_AUDADC_INT_DCMP ); //| AM_HAL_AUDADC_INT_CNVCMP | AM_HAL_AUDADC_INT_SCNCMP);
}

//*****************************************************************************
//
// Interrupt handler for the AUDADC.
//
//*****************************************************************************
void
am_audadc0_isr(void)
{
    uint32_t ui32IntMask;
    //
    // Read the interrupt status.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_interrupt_status(g_AUDADCHandle, &ui32IntMask, false))
    {
        am_util_stdio_printf("Error reading AUDADC interrupt status\n");
    }

    //
    // Clear the AUDADC interrupt.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_interrupt_clear(g_AUDADCHandle, ui32IntMask))
    {
        am_util_stdio_printf("Error clearing AUDADC interrupt status\n");
    }

    //
    // If we got a DMA complete, set the flag.
    //
    if (ui32IntMask & AM_HAL_AUDADC_INT_FIFOOVR1)
    {
        if ( AUDADCn(0)->DMASTAT_b.DMACPL )
        {
            g_bAUDADCDMAComplete = true;
        }
    }

    //
    // If we got a DMA error, set the flag.
    //
    if ( ui32IntMask & AM_HAL_AUDADC_INT_DERR )
    {
        g_bAUDADCDMAError = true;
    }
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    am_util_delay_ms(3000);

    //
    // Start the ITM interface.
    //
    am_bsp_itm_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("==============================================\n");
    am_util_stdio_printf("AUDADC RTT streaming example.\n\n");

    g_adc = (AUDADC_Type*)AUDADC_BASE;
    g_mcuctrl = (MCUCTRL_Type*)MCUCTRL_BASE;

    //
    // Initialize RTT
    //
  #if AUDADC_EXAMPLE_DEBUG
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(1, "DataLogger", g_rttRecorderBuffer, RTT_BUFFER_LENGTH, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  #endif

    //
    // Print compiler version and rtt control block address
    //
    am_util_stdio_printf("App Compiler:    %s\n", COMPILER_VERSION);
    am_util_stdio_printf("RTT Control Block Address:  0x%08X\n", (uint32_t)&_SEGGER_RTT);
    am_util_stdio_printf("Disable SWO print and starting RTT streaming...\n");
    am_util_stdio_printf("==============================================\n");
    am_bsp_debug_printf_disable();

    //
    // Configure the board for low power.
    //
    am_bsp_low_power_init();

    //
    // power up PGA
    //
    am_hal_audadc_refgen_powerup();

    am_hal_audadc_pga_powerup(0);
    am_hal_audadc_pga_powerup(1);
    am_hal_audadc_pga_powerup(2);
    am_hal_audadc_pga_powerup(3);

    am_hal_audadc_gain_set(0, 2*6);
    am_hal_audadc_gain_set(1, 2*6);
    am_hal_audadc_gain_set(2, 2*6);
    am_hal_audadc_gain_set(3, 2*6);

    //
    //  turn on mic bias
    //
    am_hal_audadc_micbias_powerup(24);

    //
    // Configure the AUDADC
    //
    audadc_config();
    //am_hal_audadc_internal_pga_config(g_AUDADCHandle, &g_sAudadcGainConfig);
    audadc_slot_config();

    NVIC_SetPriority(AUDADC0_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(AUDADC0_IRQn);
    am_hal_interrupt_master_enable();

    //
    // Trigger the AUDADC sampling for the first time manually.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_sw_trigger(g_AUDADCHandle))
    {
        am_util_stdio_printf("Error - triggering the AUDADC failed.\n");
    }

    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Go to Deep Sleep.
        //
      #if (0 == AUDADC_EXAMPLE_DEBUG)
        if (!g_bAUDADCDMAComplete)
        {
            sleep();
        }
      #endif

        // Check for DMA errors.
        //
        if (g_bAUDADCDMAError)
        {
            am_util_stdio_printf("DMA Error occured\n");
            while(1);
        }

        //
        // Check if the AUDADC DMA completion interrupt occurred.
        //
        if (g_bAUDADCDMAComplete)
        {
#if AUDADC_EXAMPLE_DEBUG

          #if 0 //fifo read.
            uint32_t        ui32SampleCount;
            ui32SampleCount = AUDADC_SAMPLE_BUF_SIZE;
            if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_samples_read(g_AUDADCHandle, false,
                                                                       g_ui32AUDADCSampleBuffer,
                                                                       &ui32SampleCount,
                                                                       &sLGSampleBuffer[0],
                                                                       &sHGSampleBuffer[0]))
            {
                am_util_stdio_printf("Error - failed to process samples.\n");
            }

            for (uint32_t sample = 0; sample < ui32SampleCount; sample++)
            {
                g_in16AudioDataBuffer[sample] = (int16_t)sLGSampleBuffer[sample].ui32Sample;
            }
          #endif

            //DMA read.
            uint32_t  ui32SampleCount = AUDADC_SAMPLE_BUF_SIZE;
            for (int i = 0; i < ui32SampleCount; i++)
            {
               g_in16AudioDataBuffer[2 * i]     = (int16_t)(g_ui32AUDADCSampleBuffer[i] & 0x0000FFF0);          // Low-gain PGA sample data.
               g_in16AudioDataBuffer[2 * i + 1] = (int16_t)((g_ui32AUDADCSampleBuffer[i] >> 16) & 0x0000FFF0);  // High-gain PGA sample data.
            }

            //copy to tmp buffer and start RTT record.
            memcpy(&g_in16SampleToRTT[g_ui32SampleToRTT], g_in16AudioDataBuffer, AUDADC_DATA_BUFFER_SIZE*sizeof(int16_t));
            g_ui32SampleToRTT += AUDADC_DATA_BUFFER_SIZE;
            if ( g_ui32SampleToRTT >= AUDIO_SAMPLE_TO_RTT - 1024 )
            {
                am_hal_audadc_disable(g_AUDADCHandle);
                pcm_rtt_record_slow(g_in16SampleToRTT, AUDIO_SAMPLE_TO_RTT*sizeof(int16_t));
            }
#endif
            //
            // Reset the DMA completion and error flags.
            //
            g_bAUDADCDMAComplete = false;

            //
            // Re-configure the AUDADC DMA.
            //
            audadc_config_dma();

            //
            // Clear the AUDADC interrupts.
            //
            if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_interrupt_clear(g_AUDADCHandle, 0xFFFFFFFF))
            {
                am_util_stdio_printf("Error - clearing the AUDADC interrupts failed.\n");
            }

        }
    }
}
