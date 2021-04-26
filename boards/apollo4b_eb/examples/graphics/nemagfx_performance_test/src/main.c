//*****************************************************************************
//
//! @file main.c
//!
//! @brief Global includes for the nemagfx app.
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

#include "main.h"
#include "am_devices_mspi_psram_aps12808l.h"

extern int32_t nemagfx_performance_test(void);

//!< TODO: Do we really need to alloc this buffer at SSRAM?
AM_SHARED_RW uint32_t        g_ui32DMATCBBuffer[2560];
void            *g_pPSRAMHandle;
void            *g_pMSPIHandle;

am_devices_mspi_psram_config_t MSPI_PSRAM_OctalCE1MSPIConfig =
{
    .eDeviceConfig            = AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1,
    .eClockFreq               = AM_HAL_MSPI_CLK_96MHZ,
    .ui32NBTxnBufLength       = sizeof(g_ui32DMATCBBuffer) / sizeof(uint32_t),
    .pNBTxnBuf                = g_ui32DMATCBBuffer,
    .ui32ScramblingStartAddr  = 0,
    .ui32ScramblingEndAddr    = 0,
};
//! MSPI interrupts.
static const IRQn_Type mspi_interrupts[] =
{
    MSPI0_IRQn,
#if defined(AM_PART_APOLLO3P) || defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    MSPI1_IRQn,
    MSPI2_IRQn,
#endif
};

//
// Take over the interrupt handler for whichever MSPI we're using.
//
#define psram_mspi_isr                                                          \
    am_mspi_isr1(MSPI_PSRAM_MODULE)
#define am_mspi_isr1(n)                                                        \
    am_mspi_isr(n)
#define am_mspi_isr(n)                                                         \
    am_mspi ## n ## _isr

//*****************************************************************************
//
// MSPI ISRs.
//
//*****************************************************************************
void
psram_mspi_isr(void)
{
    uint32_t ui32Status;

    am_hal_mspi_interrupt_status_get(g_pMSPIHandle, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_pMSPIHandle, ui32Status);

    am_hal_mspi_interrupt_service(g_pMSPIHandle, ui32Status);
}

//*****************************************************************************
//
// Main function
//
//*****************************************************************************
int32_t
main(void)
{
    uint32_t ui32Status;

    //
    // Global interrupt enable
    //
    am_hal_interrupt_master_enable();

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();
    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();
    //
    // External power on
    //
    am_bsp_external_pwr_on();
    am_util_delay_ms(100);
    //
    // Configure the MSPI and PSRAM Device.
    //
    ui32Status = am_devices_mspi_psram_aps12808l_ddr_init(MSPI_PSRAM_MODULE, &MSPI_PSRAM_OctalCE1MSPIConfig, &g_pPSRAMHandle, &g_pMSPIHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to configure the MSPI and PSRAM Device correctly!\n");
    }
    NVIC_SetPriority(mspi_interrupts[MSPI_PSRAM_MODULE], AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(mspi_interrupts[MSPI_PSRAM_MODULE]);

    //
    // Enable XIP mode.
    //
    ui32Status = am_devices_mspi_psram_aps12808l_ddr_enable_xip(g_pPSRAMHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS == ui32Status)
    {
        am_util_stdio_printf("Enable XIP mode in the MSPI!\n");
    }
    else
    {
        am_util_stdio_printf("Failed to enable XIP mode in the MSPI!\n");
    }

    //
    // If you want to change display configurations in example, please change them here as below.
    //
    // g_eDispType = RM67162_SPI4;
    // g_sDsiCfg.ui8NumLanes = 1;
    // g_sDsiCfg.eDbiWidth = AM_HAL_DSI_DBI_WIDTH_8;
    // g_sDsiCfg.eDsiFreq = AM_HAL_DSI_FREQ_TRIM_X12;
    if ((g_sDispCfg[g_eDispType].eInterface == IF_DSI) || (g_sDispCfg[g_eDispType].bUseDPHYPLL == true))
    {
        //
        // VDD18 control callback function
        //
        am_hal_dsi_register_external_vdd18_callback(am_bsp_external_vdd18_switch);
        //
        // Enable DSI power and configure DSI clock.
        //
        am_hal_dsi_init();
    }
    else
    {
        am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_HFRC96, NULL);
        am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DCCLK_ENABLE, NULL);
    }
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DISP);
    //
    // Initialize the printf interface for ITM output
    //
    am_bsp_debug_printf_enable();
    //
    // Initialize timer for time measurement
    //
    am_hal_timer_config_t sTimerConfig;
    ui32Status = am_hal_timer_default_config_set(&sTimerConfig);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to initialize a timer configuration structure with default values!\n");
    }
    sTimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16;
    sTimerConfig.eFunction = AM_HAL_TIMER_FN_REPEATPATTERN;
    ui32Status = am_hal_timer_config(0, &sTimerConfig);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to configure a timer!\n");
    }
    ui32Status = am_hal_timer_start(0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to start a timer!\n");
    }

    //
    // Run the performace test
    //
    nemagfx_performance_test();

    while (1)
    {
    }

}

