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
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "nema_hal.h"
#include "am_devices_mspi_psram_aps12808l.h"
#include "am_devices_mspi_atxp032.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define AM_DEBUG_PRINTF

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
AM_SHARED_RW uint32_t        ui32DMATCBBuffer1[2560];
AM_SHARED_RW uint32_t        ui32DMATCBBuffer2[2560];
void            *g_pMSPIFlashHandle;
void            *g_pFlashHandle;
void            *g_pPsramHandle;
void            *g_pMSPIPsramHandle;

am_devices_mspi_psram_config_t g_sMspiPsramConfig =
{
    .eDeviceConfig            = AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1,
    .eClockFreq               = AM_HAL_MSPI_CLK_96MHZ,
    .ui32NBTxnBufLength       = sizeof(ui32DMATCBBuffer2) / sizeof(uint32_t),
    .pNBTxnBuf                = ui32DMATCBBuffer2,
    .ui32ScramblingStartAddr  = 0,
    .ui32ScramblingEndAddr    = 0,
};

am_devices_mspi_atxp032_config_t g_sMspiFlashConfig =
{
    .eDeviceConfig            = AM_HAL_MSPI_FLASH_QUAD_CE0,
    .eClockFreq               = AM_HAL_MSPI_CLK_48MHZ,
    .pNBTxnBuf                = ui32DMATCBBuffer1,
    .ui32NBTxnBufLength       = (sizeof(ui32DMATCBBuffer1) / sizeof(uint32_t)),
    .ui32ScramblingStartAddr  = 0,
    .ui32ScramblingEndAddr    = 0,
};


//! MSPI interrupts.
static const IRQn_Type MspiInterrupts[] =
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


#define flash_mspi_isr                                                          \
    am_mspi_isr1(MSPI_FLASH_MODULE)
#define am_mspi_isr1(n)                                                        \
    am_mspi_isr(n)
#define am_mspi_isr(n)                                                         \
    am_mspi ## n ## _isr
//*****************************************************************************
//
// MSPI ISRs.
//
//*****************************************************************************
void psram_mspi_isr(void)
{
   uint32_t      ui32Status;

#ifdef SYSTEM_VIEW
   traceISR_ENTER();
#endif

   am_hal_mspi_interrupt_status_get(g_pMSPIPsramHandle, &ui32Status, false);

   am_hal_mspi_interrupt_clear(g_pMSPIPsramHandle, ui32Status);

   am_hal_mspi_interrupt_service(g_pMSPIPsramHandle, ui32Status);

#ifdef SYSTEM_VIEW
   traceISR_EXIT();
#endif
}

void flash_mspi_isr(void)
{
    uint32_t      ui32Status;

#ifdef SYSTEM_VIEW
   traceISR_ENTER();
#endif

    am_hal_mspi_interrupt_status_get(g_pMSPIFlashHandle, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_pMSPIFlashHandle, ui32Status);

    am_hal_mspi_interrupt_service(g_pMSPIFlashHandle, ui32Status);

#ifdef SYSTEM_VIEW
   traceISR_EXIT();
#endif
}

//*****************************************************************************
//
// Enable printing to the console.
//
//*****************************************************************************
void
enable_print_interface(void)
{
    //
    // Initialize a debug printing interface.
    //
    am_bsp_debug_printf_enable();
}

//*****************************************************************************
//
// Disable printing to the console.
//
//*****************************************************************************
void
disable_print_interface(void)
{
    //
    // Initialize a debug printing interface.
    //
    am_bsp_debug_printf_disable();
    am_hal_itm_disable();
}

#ifdef BAREMETAL
extern int watchface(void);
#endif

//*****************************************************************************
//
// Main Function
//
//*****************************************************************************
int
main(void)
{
    uint32_t      ui32Status;

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();

    //
    // Configure the SEGGER SystemView Interface.
    //
#ifdef SYSTEM_VIEW
    SEGGER_SYSVIEW_Conf();
#endif

    //
    // Power up extended memory.
    //
    am_hal_pwrctrl_dsp_memory_config_t cfg = g_DefaultDSPMemCfg;
    am_hal_pwrctrl_dsp_memory_config(AM_HAL_DSP0, &cfg);
    am_hal_pwrctrl_dsp_memory_config(AM_HAL_DSP1, &cfg);

    //
    // Configure the MSPI and PSRAM Device.
    //
    ui32Status = am_devices_mspi_psram_aps12808l_ddr_init(MSPI_PSRAM_MODULE, &g_sMspiPsramConfig, &g_pPsramHandle, &g_pMSPIPsramHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to configure the MSPI and PSRAM Device correctly!\n");
    }
    #ifndef BAREMETAL
    NVIC_SetPriority(MspiInterrupts[MSPI_PSRAM_MODULE], NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
    #endif
    NVIC_EnableIRQ(MspiInterrupts[MSPI_PSRAM_MODULE]);

    //
    // Configure the MSPI and Flash Device.
    //
    ui32Status = am_devices_mspi_atxp032_init(MSPI_FLASH_MODULE, (void*)&g_sMspiFlashConfig, &g_pFlashHandle, &g_pMSPIFlashHandle);
    if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to configure the MSPI and Flash Device correctly!\n");
    }
    #ifndef BAREMETAL
    NVIC_SetPriority(MspiInterrupts[MSPI_FLASH_MODULE], NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
    #endif
    NVIC_EnableIRQ(MspiInterrupts[MSPI_FLASH_MODULE]);

    //
    // Enable global IRQ.
    //
    am_hal_interrupt_master_enable();

    //
    // Enable XIP mode.
    //
    ui32Status = am_devices_mspi_psram_aps12808l_ddr_enable_xip(g_pPsramHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS == ui32Status)
    {
        am_util_stdio_printf("Enable XIP mode in the MSPI!\n");
    }
    else
    {
        am_util_stdio_printf("Failed to enable XIP mode in the MSPI!\n");
    }

    //
    // Make sure we aren't in XIP mode.
    //
    ui32Status = am_devices_mspi_atxp032_disable_xip(g_pFlashHandle);
    if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to disable XIP mode in the MSPI!\n");
    }

    //
    // If you want to change display configurations in example, please change them here as below.
    //
    // g_eDispType = RM67162_SPI4;
    // g_sDsiCfg.ui8NumLanes = 1;
    // g_sDsiCfg.eDbiWidth = AM_HAL_DSI_DBI_WIDTH_8;
    // g_sDsiCfg.eDsiFreq = AM_HAL_DSI_FREQ_TRIM_X12;

    if (g_sDispCfg[g_eDispType].eInterface == IF_DSI)
    {
        //
        // Enable DSI power and configure DSI clock.
        //
        am_hal_dsi_init();
    }
    else
    {
        if (g_sDispCfg[g_eDispType].bUseDPHYPLL == true)
        {
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_DPHYPLL, NULL);
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DCCLK_ENABLE, NULL);
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_HFRC12, NULL);
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLK_ENABLE, NULL);
        }
        else
        {
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_HFRC96, NULL);
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DCCLK_ENABLE, NULL);
        }
    }
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DISP);
    if (g_sDispCfg[g_eDispType].bUseDPHYPLL == true)
    {
        am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DISPPHY);
    }

    //
    // Initialize the printf interface for ITM output
    //
    am_bsp_debug_printf_enable();

    //
    // Enable printing to the console.
    //
#ifdef AM_DEBUG_PRINTF
    enable_print_interface();
#endif

    CPU->DAXICFG_b.BUFFERENABLE = CPU_DAXICFG_BUFFERENABLE_ONE;  // only enable a single buffer

    PWRCTRL->SSRAMPWREN = PWRCTRL_SSRAMPWREN_PWRENSSRAM_ALL;  // enable all
    while(PWRCTRL->SSRAMPWRST == 0);

    //
    // Initialize timer
    //
    am_hal_timer_config_t       TimerConfig;
    am_hal_timer_default_config_set(&TimerConfig);
    TimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16;
    TimerConfig.eFunction = AM_HAL_TIMER_FN_EDGE;

    am_hal_timer_config(0, &TimerConfig);
    am_hal_timer_start(0);

    //
    // Initialize plotting interface.
    //
    am_util_debug_printf("watchface Example\n");

#ifdef BAREMETAL
    //
    // Run the watchface func.
    //
    watchface();
#else /* BAREMETAL */

    //
    // Run the application.
    //
    run_tasks();
#endif /* BAREMETAL*/

    //
    // We shouldn't ever get here.
    //
    while (1)
    {
    }

}

