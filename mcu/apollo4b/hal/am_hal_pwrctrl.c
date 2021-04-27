//*****************************************************************************
//
//  am_hal_pwrctrl.c
//! @file
//!
//! @brief Functions for enabling and disabling power domains.
//!
//! @addtogroup pwrctrl4 Power Control
//! @ingroup apollo4hal
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
#include "am_mcu_apollo.h"

//
// Maximum number of checks to memory power status before declaring error
// (5 x 1usec = 5usec).
//
#define AM_HAL_PWRCTRL_MAX_WAIT_US      5
#define AM_HAL_PWRCTRL_MEMPWREN_MASK    ( PWRCTRL_MEMPWREN_PWRENDTCM_Msk        |   \
                                          PWRCTRL_MEMPWREN_PWRENNVM0_Msk        |   \
                                          PWRCTRL_MEMPWREN_PWRENCACHEB0_Msk     |   \
                                          PWRCTRL_MEMPWREN_PWRENCACHEB2_Msk )

#define AM_HAL_PWRCTRL_DSPMEMPWRST_MASK ( PWRCTRL_DSP0MEMPWRST_PWRSTDSP0RAM_Msk |   \
                                          PWRCTRL_DSP0MEMPWRST_PWRSTDSP0ICACHE_Msk )

//
// Defines to enable Apollo4 RevB workarounds.
//
#define AM_HAL_PWRCTL_HPLP_WA
#define AM_HAL_PWRCTL_CRYPTO_WA

#ifdef AM_HAL_PWRCTL_CRYPTO_WA
#define AM_HAL_PWRCTL_CRYPTO_DELAY 16
#endif

#ifdef AM_HAL_PWRCTL_HPLP_WA
#define AM_HAL_PWRCTL_HPLP_DELAY   10
#endif

//
// Define max values of some particular fields
//
#define MAX_BUCKVDDFTRIM    _FLD2VAL(MCUCTRL_SIMOBUCK12_SIMOBUCKACTTRIMVDDF, MCUCTRL_SIMOBUCK12_SIMOBUCKACTTRIMVDDF_Msk)
#define MAX_LDOVDDFTRIM     _FLD2VAL(MCUCTRL_LDOREG2_MEMLDOACTIVETRIM, MCUCTRL_LDOREG2_MEMLDOACTIVETRIM_Msk)
#define MAX_VDDCTRIM        _FLD2VAL(MCUCTRL_VREFGEN2_TVRGVREFTRIM, MCUCTRL_VREFGEN2_TVRG2TEMPCOTRIM_Msk)

//
// Global State Variables for the VDDF and VDDC boosting
//
am_hal_pwrctrl_mcu_mode_e g_eCurrPwrMode = AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER;
uint32_t g_ui32TrimVer              = 0xFFFFFFFF;

uint32_t g_ui32origSimobuckVDDFtrim = 0xFFFFFFFF;
uint32_t g_ui32origSimobuckVDDCtrim = 0xFFFFFFFF;
uint32_t g_ui32origLDOVDDCtrim      = 0xFFFFFFFF;
bool     g_bVDDCbuckboosted = false;
bool     g_bVDDCLDOboosted  = false;

//
// Define the peripheral control structure.
//
const struct
{
  uint32_t      ui32PwrEnRegAddr;
  uint32_t      ui32PeriphEnable;
  uint32_t      ui32PwrStatReqAddr;
  uint32_t      ui32PeriphStatus;
}

//
// Peripheral control data structure
//
am_hal_pwrctrl_peripheral_control[AM_HAL_PWRCTRL_PERIPH_MAX] =
{
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOS, PWRCTRL_DEVPWREN_PWRENIOS_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOS_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM0, PWRCTRL_DEVPWREN_PWRENIOM0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOM0_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM1, PWRCTRL_DEVPWREN_PWRENIOM1_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOM1_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM2, PWRCTRL_DEVPWREN_PWRENIOM2_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOM2_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM3, PWRCTRL_DEVPWREN_PWRENIOM3_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOM3_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM4, PWRCTRL_DEVPWREN_PWRENIOM4_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOM4_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM5, PWRCTRL_DEVPWREN_PWRENIOM5_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOM5_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM6, PWRCTRL_DEVPWREN_PWRENIOM6_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOM6_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM7, PWRCTRL_DEVPWREN_PWRENIOM7_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOM7_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART0, PWRCTRL_DEVPWREN_PWRENUART0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTUART0_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART1, PWRCTRL_DEVPWREN_PWRENUART1_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTUART1_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART2, PWRCTRL_DEVPWREN_PWRENUART2_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTUART2_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART3, PWRCTRL_DEVPWREN_PWRENUART3_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTUART3_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENADC, PWRCTRL_DEVPWREN_PWRENADC_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTADC_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI0, PWRCTRL_DEVPWREN_PWRENMSPI0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTMSPI0_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI1, PWRCTRL_DEVPWREN_PWRENMSPI1_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTMSPI1_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI2, PWRCTRL_DEVPWREN_PWRENMSPI2_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTMSPI2_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENGFX, PWRCTRL_DEVPWREN_PWRENGFX_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTGFX_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISP, PWRCTRL_DEVPWREN_PWRENDISP_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTDISP_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISPPHY, PWRCTRL_DEVPWREN_PWRENDISPPHY_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTDISPPHY_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENCRYPTO, PWRCTRL_DEVPWREN_PWRENCRYPTO_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTCRYPTO_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENSDIO, PWRCTRL_DEVPWREN_PWRENSDIO_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTSDIO_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSB, PWRCTRL_DEVPWREN_PWRENUSB_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTUSB_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSBPHY, PWRCTRL_DEVPWREN_PWRENUSBPHY_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTUSBPHY_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDBG, PWRCTRL_DEVPWREN_PWRENDBG_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTDBG_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDREC, PWRCTRL_AUDSSPWREN_PWRENAUDREC_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTAUDREC_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDPB, PWRCTRL_AUDSSPWREN_PWRENAUDPB_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTAUDPB_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM0, PWRCTRL_AUDSSPWREN_PWRENPDM0_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTPDM0_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM1, PWRCTRL_AUDSSPWREN_PWRENPDM1_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTPDM1_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM2, PWRCTRL_AUDSSPWREN_PWRENPDM2_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTPDM2_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM3, PWRCTRL_AUDSSPWREN_PWRENPDM3_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTPDM3_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENI2S0, PWRCTRL_AUDSSPWREN_PWRENI2S0_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTI2S0_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENI2S1, PWRCTRL_AUDSSPWREN_PWRENI2S1_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTI2S1_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDADC, PWRCTRL_AUDSSPWREN_PWRENAUDADC_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTAUDADC_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENDSPA, PWRCTRL_AUDSSPWREN_PWRENDSPA_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        PWRCTRL_AUDSSPWREN_PWRENDSPA_Msk
    },
};

//*****************************************************************************
//
// Default configurations.
//
//*****************************************************************************
const am_hal_pwrctrl_mcu_memory_config_t      g_DefaultMcuMemCfg =
{
    .eCacheCfg          = AM_HAL_PWRCTRL_CACHE_ALL,
    .bRetainCache       = true,
    .eDTCMCfg           = AM_HAL_PWRCTRL_DTCM_384K,
    .eRetainDTCM        = AM_HAL_PWRCTRL_DTCM_384K,
    .bEnableNVM0        = true,
    .bRetainNVM0        = false
};

const am_hal_pwrctrl_sram_memcfg_t            g_DefaultSRAMCfg =
{
    .eSRAMCfg           = AM_HAL_PWRCTRL_SRAM_1M,
    .eActiveWithMCU     = AM_HAL_PWRCTRL_SRAM_NONE,
    .eActiveWithDSP     = AM_HAL_PWRCTRL_SRAM_NONE,
    .eSRAMRetain        = AM_HAL_PWRCTRL_SRAM_1M
};

const am_hal_pwrctrl_dsp_memory_config_t      g_DefaultDSPMemCfg =
{
    .bEnableICache      = true,
    .bRetainCache       = true,
    .bEnableRAM         = true,
    .bActiveRAM         = false,
    .bRetainRAM         = true
};

#ifdef AM_HAL_PWRCTL_CRYPTO_WA
// ****************************************************************************
//
//  crypto_powerdown
//  Select the MCU power mode.
//
// ****************************************************************************
#define CRYPTO_WAIT_USEC        100
static
uint32_t crypto_powerdown(void)

{
    uint32_t      ui32Status;

    //
    // Wait for crypto block to go idle.
    //
    ui32Status = am_hal_delay_us_status_change(CRYPTO_WAIT_USEC,
                                               (uint32_t)&CRYPTO->HOSTCCISIDLE,
                                               CRYPTO_HOSTCCISIDLE_HOSTCCISIDLE_Msk,
                                               CRYPTO_HOSTCCISIDLE_HOSTCCISIDLE_Msk);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Wait for OTP to go idle.
    //
    ui32Status = am_hal_delay_us_status_change(CRYPTO_WAIT_USEC,
                                               (uint32_t)&CRYPTO->NVMISIDLE,
                                               CRYPTO_NVMISIDLE_NVMISIDLEREG_Msk,
                                               CRYPTO_NVMISIDLE_NVMISIDLEREG_Msk);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Power down the crypto block.
    //
    CRYPTO->HOSTPOWERDOWN_b.HOSTPOWERDOWN = 1;

    //
    // Wait (just in case).
    //
    am_hal_delay_us(15);

    return AM_HAL_STATUS_SUCCESS;
} // crypto_powerdown()
#endif // AM_HAL_PWRCTL_CRYPTO_WA

// ****************************************************************************
//
//  HP_mode
//  Select the performance mode.
//
// ****************************************************************************

static uint32_t g_WaitTimerDuration = 0;

// Using internal non-published function - as Timer13 is reserved for workaround
extern uint32_t internal_timer_config(uint32_t ui32TimerNumber,
                                      am_hal_timer_config_t *psTimerConfig);

//*****************************************************************************
//
// Function to determine the chip's TRIM version.
//
//*****************************************************************************
static uint32_t
TrimVersionGet(uint32_t *pui32TrimVer)
{
    uint32_t ui32Ret;

    //
    // Get the TRIM version and set the global variable.
    // This only needs to be done and verified once.
    //
    if ( g_ui32TrimVer == 0xFFFFFFFF )
    {
        ui32Ret = am_hal_mram_info_read(1, AM_REG_INFO1_TRIM_REV_O / 4, 1, &g_ui32TrimVer);

        if ( (ui32Ret != 0) || (g_ui32TrimVer == 0xFFFFFFFF) )
        {
            //
            // Invalid trim value. Set the global to indicate version 0.
            //
            g_ui32TrimVer = 0;
        }
    }

    if ( pui32TrimVer )
    {
        *pui32TrimVer = g_ui32TrimVer;
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

} // TrimVersionGet()


//*****************************************************************************
//
// Function to initialize Timer 13 to interrupt after ui32Delayus.
//
//*****************************************************************************
static uint32_t
am_hal_util_write_and_wait_timer_init(uint32_t ui32Delayus)
{
    am_hal_timer_config_t       TimerConfig;
    uint32_t ui32Status         = AM_HAL_STATUS_SUCCESS;

    //
    // Set the timer configuration
    //
    am_hal_timer_default_config_set(&TimerConfig);
    TimerConfig.eFunction = AM_HAL_TIMER_FN_EDGE;
    TimerConfig.ui32Compare0 = 0xFFFFFFFF;
    TimerConfig.ui32PatternLimit = 0;
    TimerConfig.ui32Compare1 = ui32Delayus * 6000000 / 1000000;
    ui32Status = internal_timer_config(AM_HAL_WRITE_WAIT_TIMER, &TimerConfig);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
       return ui32Status;
    }

    am_hal_timer_clear(AM_HAL_WRITE_WAIT_TIMER);
    am_hal_timer_stop(AM_HAL_WRITE_WAIT_TIMER);

    //
    // Clear the timer Interrupt
    //
    ui32Status = am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(AM_HAL_WRITE_WAIT_TIMER, AM_HAL_TIMER_COMPARE1));
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    // Enable the timer Interrupt.
    //
    ui32Status = am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(AM_HAL_WRITE_WAIT_TIMER, AM_HAL_TIMER_COMPARE1));
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
       return ui32Status;
    }

    //
    // Enable the timer interrupt in the NVIC.
    //
    // This interrupt needs to be set as the highest priority (0)
    //
    NVIC_SetPriority((IRQn_Type)((uint32_t)TIMER0_IRQn + AM_HAL_WRITE_WAIT_TIMER), 0);
    NVIC_EnableIRQ((IRQn_Type)((uint32_t)TIMER0_IRQn + AM_HAL_WRITE_WAIT_TIMER));

    //
    // No need to enable interrupt as we just need to get out of WFI
    // We just need to clear the NVIC pending
    // am_hal_interrupt_master_enable();
    //
    g_WaitTimerDuration = ui32Delayus;

    return ui32Status;

} // am_hal_util_write_and_wait_timer_init()


//*****************************************************************************
//
// Define a simple function that will write a value to register (or other
// memory location) and then go to sleep.
// The opcodes are aligned on a 16-byte boundary to guarantee that
// the write and the WFI are in the same M4 line buffer.
//
//*****************************************************************************
//
// Prototype the assembly function.
//
typedef void (*storeAndWFIfunc_t)(uint32_t ui32Val, uint32_t *pAddr);

#if 0
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
__align(16)
#define WA_ATTRIB
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
#warning This attribute is not yet tested on ARM6.
#define WA_ATTRIB   __attribute__ ((aligned (16)))
#elif defined(__GNUC_STDC_INLINE__)
#define WA_ATTRIB   __attribute__ ((aligned (16)))
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 16
#define WA_ATTRIB
#else
#error Unknown compiler.
#endif

static const
uint16_t storeAndWFI[16] WA_ATTRIB =
{
    //
    // r0: Value to be written to the location specified in the 2nd argument.
    // r1: Address of the location to be written.
    // r2: 0x0 or addr of the SYNC_READ reg (performs a dummy read if non-zero)
    //
    // Begin 1st line buffer
    0x6008,             // str r0, [r1]
    0xF3BF, 0x8F4F,     // DSB
    0xBF30,             // WFI
    0xF3BF, 0x8F6F,     // ISB
    0x4770,             // bx lr
    0xBF00,             // nop
};
storeAndWFIfunc_t storeAndWFIfunc = (storeAndWFIfunc_t)((uint8_t *)storeAndWFI + 1);

#endif

#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
__align(16)
#define WA_ATTRIB
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
#warning This attribute is not yet tested on ARM6.
#define WA_ATTRIB   __attribute__ ((aligned (16)))
#elif defined(__GNUC_STDC_INLINE__)
#define WA_ATTRIB   __attribute__ ((aligned (16)))
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 16
#define WA_ATTRIB
#else
#error Unknown compiler.
#endif

static
uint16_t storeAndWFIRAM[16] WA_ATTRIB =
{
    //
    // r0: Value to be written to the location specified in the 2nd argument.
    // r1: Address of the location to be written.
    // r2: 0x0 or addr of the SYNC_READ reg (performs a dummy read if non-zero)
    //
    // Begin 1st line buffer
    0x6008,             // str r0, [r1]
    0xF3BF, 0x8F4F,     // DSB
    0xBF30,             // WFI
    0xF3BF, 0x8F6F,     // ISB
    0x4770,             // bx lr
    0xBF00,             // nop
};


//
// Prototype the assembly function.
//
storeAndWFIfunc_t storeAndWFIfuncRAM = (storeAndWFIfunc_t)((uint8_t *)storeAndWFIRAM + 1);

//*****************************************************************************
//
// am_hal_util_write_and_wait()
// Function to perform the RevB HP/LP mode switch.
//
//*****************************************************************************
uint32_t
am_hal_util_write_and_wait(uint32_t *pAddr, uint32_t ui32Mask, uint32_t ui32Val, uint32_t ui32Delayus)

{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t origBasePri;
    uint32_t basePrioGrouping;

    if ( ui32Delayus != g_WaitTimerDuration )
    {
        ui32Status = am_hal_util_write_and_wait_timer_init(ui32Delayus);
        if (ui32Status != AM_HAL_STATUS_SUCCESS)
        {
            return ui32Status;
        }
    }

    basePrioGrouping = NVIC_GetPriorityGrouping();
    if (basePrioGrouping == 7)
    {
        //
        // We cannot implement this workaround
        //
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Begin critical section
    //
    AM_CRITICAL_BEGIN

    //
    // Mask off all other interrupts
    //
    origBasePri = __get_BASEPRI();
    if (basePrioGrouping >= (8 - __NVIC_PRIO_BITS))
    {
        __set_BASEPRI(1 << (basePrioGrouping + 1));
    }
    else
    {
        __set_BASEPRI(1 << (8 - __NVIC_PRIO_BITS));
    }

    //
    // Compute the value to write
    //
    if (ui32Mask != 0xFFFFFFFF)
    {
        ui32Val |= (AM_REGVAL((uint32_t)pAddr) & ~ui32Mask);
    }

    //
    // Clear the timer. This HAL function also enables the timer via AUXEN.
    //
    am_hal_timer_clear(AM_HAL_WRITE_WAIT_TIMER);

    //
    // Set for normal sleep before calling storeAndWFIfunc()
    //
    SCB->SCR &= ~_VAL2FLD(SCB_SCR_SLEEPDEEP, 1);

    //
    // Call the function to switch the performance mode and WFI.
    //
    storeAndWFIfuncRAM(ui32Val, pAddr);

    //
    // Stop/Disable the timer
    //
    am_hal_timer_stop(AM_HAL_WRITE_WAIT_TIMER);

    //
    // Clear the timer Interrupt
    //
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(AM_HAL_WRITE_WAIT_TIMER, AM_HAL_TIMER_COMPARE1));

    //
    // Make sure the interrupt has been cleared before clearing the NVIC pending to prevent race condition
    //
    uint32_t dummy;
    am_hal_timer_interrupt_status_get(false, &dummy);

    //
    // am_hal_sysctrl_bus_write_flush();
    //
    NVIC_ClearPendingIRQ((IRQn_Type)((uint32_t)TIMER0_IRQn + AM_HAL_WRITE_WAIT_TIMER));

    //
    // Restore interrupts
    //
    __set_BASEPRI(origBasePri);

    //
    // End critical section
    //
    AM_CRITICAL_END

    return ui32Status;

} // am_hal_util_write_and_wait()


typedef struct
{
    am_hal_pwrctrl_mcu_mode_e ePowerMode;
    am_hal_pwrctrl_mcu_mode_e ePowerModeOrig;
    uint32_t ui32StepMask;
    uint32_t MISC_USEHFRC2FQ96MHZ;
    uint32_t MISC_USEHFRC2FQ192MHZ;
    uint32_t PWRSW1_USEVDDF4VDDRCPUINHP;
} pwrctrl_state_restore_t;

static void
backout_mcu_mode_select(pwrctrl_state_restore_t *psSettings)
{
    //
    // Back out in reverse order of the setting.
    //
    if ( psSettings->ui32StepMask & 0x0004 )
    {
        am_hal_util_write_and_wait((uint32_t*)&PWRCTRL->MCUPERFREQ, 0xFFFFFFFF,
                                   (uint32_t)psSettings->ePowerModeOrig,
                                   AM_HAL_PWRCTL_HPLP_DELAY);
    }

    if ( psSettings->ui32StepMask & 0x0002 )
    {
        AM_CRITICAL_BEGIN

        //
        // Restore the original trim values
        //
        MCUCTRL->SIMOBUCK12_b.SIMOBUCKACTTRIMVDDF = g_ui32origSimobuckVDDFtrim;

        //
        // Delay to give voltage supply some time to transition to the new level
        //
        am_hal_delay_us(AM_HAL_PWRCTRL_VDDF_BOOST_DELAY);

        AM_CRITICAL_END
    }

    if ( psSettings->ui32StepMask & 0x0001 )
    {
        // B0 workaround: Reset the registers to their original values.
        CLKGEN->MISC_b.USEHFRC2FQ192MHZ       = psSettings->MISC_USEHFRC2FQ192MHZ;
        CLKGEN->MISC_b.USEHFRC2FQ96MHZ        = psSettings->MISC_USEHFRC2FQ96MHZ;
        MCUCTRL->PWRSW1_b.USEVDDF4VDDRCPUINHP = psSettings->PWRSW1_USEVDDF4VDDRCPUINHP;
    }

    //
    // Delay after making the settings.
    //
    am_hal_delay_us(10);
} // backout_mcu_mode_select()


// ****************************************************************************
//
//  am_hal_pwrctrl_mcu_mode_select()
//  Select the MCU power mode.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_mode_select(am_hal_pwrctrl_mcu_mode_e ePowerMode)
{
    bool bApollo4B0, bDoVDDFboost;
    uint32_t ui32Status, ui32TrimVer;
    pwrctrl_state_restore_t sSettings =
    {
        .ePowerMode = ePowerMode,
        .ePowerModeOrig = (am_hal_pwrctrl_mcu_mode_e)PWRCTRL->MCUPERFREQ_b.MCUPERFREQ,
        .ui32StepMask = 0,
        .MISC_USEHFRC2FQ96MHZ = 0,
        .MISC_USEHFRC2FQ192MHZ = 0,
        .PWRSW1_USEVDDF4VDDRCPUINHP = 0
    };

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( (ePowerMode != AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER)      &&
         (ePowerMode != AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // We must be using SIMOBUCK in order to go to HP mode.
    //
    if ( (ePowerMode == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE)   &&
         (PWRCTRL->VRSTATUS_b.SIMOBUCKST != PWRCTRL_VRSTATUS_SIMOBUCKST_ACT) )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( ePowerMode == g_eCurrPwrMode )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    g_eCurrPwrMode = ePowerMode;

    TrimVersionGet(&ui32TrimVer);
    bDoVDDFboost = ( APOLLO4_GE_B1  &&  (ui32TrimVer >= 6)  ) ? true : false;
    bApollo4B0   = APOLLO4_B0;

    if ( bApollo4B0 )
    {
        //
        // Get current state
        //
        sSettings.ui32StepMask |= 0x0001;
        sSettings.MISC_USEHFRC2FQ192MHZ = CLKGEN->MISC_b.USEHFRC2FQ192MHZ;
        sSettings.MISC_USEHFRC2FQ96MHZ  = CLKGEN->MISC_b.USEHFRC2FQ96MHZ;
        sSettings.PWRSW1_USEVDDF4VDDRCPUINHP = MCUCTRL->PWRSW1_b.USEVDDF4VDDRCPUINHP;

        //
        // Check if we need to apply a workaround for 192MHz operation.
        //
        if ( ePowerMode == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE )
        {
            CLKGEN->MISC_b.USEHFRC2FQ192MHZ = 1;
            MCUCTRL->PWRSW1_b.USEVDDF4VDDRCPUINHP = 1;
        }
        else
        {
            CLKGEN->MISC_b.USEHFRC2FQ96MHZ = 1;
        }

        //
        // Delay after setting the HFRC2 mode.
        //
        am_hal_delay_us(10);
    }

    //
    // Set the MCU power mode.
    //
    if ( bDoVDDFboost )
    {
        if ( g_ui32origSimobuckVDDFtrim == 0xFFFFFFFF )
        {
            //
            // Get and save the original value the very first time.
            //
            g_ui32origSimobuckVDDFtrim = MCUCTRL->SIMOBUCK12_b.SIMOBUCKACTTRIMVDDF;
        }

        if ( ePowerMode == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE )
        {
            uint32_t ui32newval;

            //
            // Increase VDDF up by 7 steps (or max it out).
            //
            sSettings.ui32StepMask |= 0x0002;

            //
            // SIMOBUCK trim adjustment
            //
            MCUCTRL->SIMOBUCK15_b.SIMOBUCKTRIMLATCHOVER = 1;

            ui32newval = g_ui32origSimobuckVDDFtrim <= (MAX_BUCKVDDFTRIM - 7) ?
                            g_ui32origSimobuckVDDFtrim + 7 : MAX_BUCKVDDFTRIM;

            AM_CRITICAL_BEGIN
            MCUCTRL->SIMOBUCK12_b.SIMOBUCKACTTRIMVDDF = ui32newval;

            //
            // Delay to give voltage supply some time to transition to the new level
            //
            am_hal_delay_us(AM_HAL_PWRCTRL_VDDF_BOOST_DELAY);
            AM_CRITICAL_END
        }
    }

#ifdef AM_HAL_PWRCTL_HPLP_WA
    ui32Status = am_hal_util_write_and_wait((uint32_t*)&PWRCTRL->MCUPERFREQ,
                                            0xFFFFFFFF, (uint32_t)ePowerMode,
                                            AM_HAL_PWRCTL_HPLP_DELAY);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        backout_mcu_mode_select(&sSettings);
        return ui32Status;
    }
    sSettings.ui32StepMask |= 0x0004;
#else
    PWRCTRL->MCUPERFREQ_b.MCUPERFREQ = ePowerMode;
#endif

    //
    // Wait for the ACK
    //
    ui32Status = AM_HAL_STATUS_TIMEOUT;
    for ( uint32_t i = 0; i < 5; i++ )
    {
        if ( PWRCTRL->MCUPERFREQ_b.MCUPERFACK > 0 )
        {
            ui32Status = AM_HAL_STATUS_SUCCESS;
            break;
        }
        am_hal_delay_us(1);
    }

    //
    // Check for timeout.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        backout_mcu_mode_select(&sSettings);
        return ui32Status;
    }

    if ( bDoVDDFboost  &&  (ePowerMode == AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER) )
    {
        AM_CRITICAL_BEGIN

        //
        // Restore the original SIMOBUCK trim
        //
        MCUCTRL->SIMOBUCK12_b.SIMOBUCKACTTRIMVDDF = g_ui32origSimobuckVDDFtrim;

        //
        // Delay to give voltage supply some time to transition to the new level
        //
        am_hal_delay_us(AM_HAL_PWRCTRL_VDDF_BOOST_DELAY);
        AM_CRITICAL_END
    }

    if ( bApollo4B0 )
    {
        //
        // Read the current power mode.
        //
        //ui32PowerMode = PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS;

        //
        // Check if we need to apply a workaround for 192MHz operation.
        //
        if ( AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE == ePowerMode )
        {
            CLKGEN->MISC_b.USEHFRC2FQ192MHZ = 0;
        }
        else
        {
            CLKGEN->MISC_b.USEHFRC2FQ96MHZ = 0;
        }

        //
        // Delay after setting the HFRC2 mode.
        //
        am_hal_delay_us(10);
    }

    //
    // Check the MCU power mode status and return SUCCESS/FAIL.
    //
    if ( PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS == ePowerMode )
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        backout_mcu_mode_select(&sSettings);
        return AM_HAL_STATUS_FAIL;
    }

} // am_hal_pwrctrl_mcu_mode_select()

// ****************************************************************************
//
//  am_hal_pwrctrl_mcu_memory_config()
//  Configure the MCU memory.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_memory_config(am_hal_pwrctrl_mcu_memory_config_t *psConfig)
{
    uint32_t      ui32Status;

    //
    // Configure the MCU Cache.
    //
    switch ( psConfig->eCacheCfg )
    {
        case AM_HAL_PWRCTRL_CACHE_NONE:
            PWRCTRL->MEMPWREN_b.PWRENCACHEB0 = PWRCTRL_MEMPWREN_PWRENCACHEB0_DIS;
            PWRCTRL->MEMPWREN_b.PWRENCACHEB2 = PWRCTRL_MEMPWREN_PWRENCACHEB2_DIS;
            break;
        case AM_HAL_PWRCTRL_CACHEB0_ONLY:
            PWRCTRL->MEMPWREN_b.PWRENCACHEB0 = PWRCTRL_MEMPWREN_PWRENCACHEB0_EN;
            PWRCTRL->MEMPWREN_b.PWRENCACHEB2 = PWRCTRL_MEMPWREN_PWRENCACHEB2_DIS;
            break;
        case AM_HAL_PWRCTRL_CACHE_ALL:
            PWRCTRL->MEMPWREN_b.PWRENCACHEB0 = PWRCTRL_MEMPWREN_PWRENCACHEB0_EN;
            PWRCTRL->MEMPWREN_b.PWRENCACHEB2 = PWRCTRL_MEMPWREN_PWRENCACHEB2_EN;
            break;
    }

    //
    // Configure the MCU Tightly Coupled Memory.
    //
    PWRCTRL->MEMPWREN_b.PWRENDTCM = psConfig->eDTCMCfg;

    //
    // Configure the Non-Volatile Memory.
    //
    PWRCTRL->MEMPWREN_b.PWRENNVM0 = psConfig->bEnableNVM0;

    DIAG_SUPPRESS_VOLATILE_ORDER()
    //
    // Wait for Status
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              (uint32_t)&PWRCTRL->MEMPWRSTATUS,
                                              AM_HAL_PWRCTRL_MEMPWREN_MASK,
                                              PWRCTRL->MEMPWREN,
                                              true);

    //
    // Check for timeout.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check the MCU power mode status and return SUCCESS/FAIL.
    //
    if ((PWRCTRL->MEMPWRSTATUS_b.PWRSTCACHEB0 != PWRCTRL->MEMPWREN_b.PWRENCACHEB0)  ||
        (PWRCTRL->MEMPWRSTATUS_b.PWRSTCACHEB2 != PWRCTRL->MEMPWREN_b.PWRENCACHEB2)  ||
        (PWRCTRL->MEMPWRSTATUS_b.PWRSTDTCM != PWRCTRL->MEMPWREN_b.PWRENDTCM)        ||
        (PWRCTRL->MEMPWRSTATUS_b.PWRSTNVM0 != PWRCTRL->MEMPWREN_b.PWRENNVM0))
    {
        return AM_HAL_STATUS_FAIL;
    }

    DIAG_DEFAULT_VOLATILE_ORDER()

    //
    // Configure Cache retention.
    //
    if (psConfig->bRetainCache)
    {
        PWRCTRL->MEMRETCFG_b.CACHEPWDSLP = PWRCTRL_MEMRETCFG_CACHEPWDSLP_DIS;
    }
    else
    {
        PWRCTRL->MEMRETCFG_b.CACHEPWDSLP = PWRCTRL_MEMRETCFG_CACHEPWDSLP_EN;
    }

    //
    // Configure the Non-Volatile Memory retention.
    //
    if (psConfig->bRetainNVM0)
    {
        PWRCTRL->MEMRETCFG_b.NVM0PWDSLP = PWRCTRL_MEMRETCFG_NVM0PWDSLP_DIS;
    }
    else
    {
        PWRCTRL->MEMRETCFG_b.NVM0PWDSLP = PWRCTRL_MEMRETCFG_NVM0PWDSLP_EN;
    }

    //
    // Configure the MCU Tightly Coupled Memory retention.
    //
    switch ( psConfig->eRetainDTCM )
    {
        case AM_HAL_PWRCTRL_DTCM_NONE:
            PWRCTRL->MEMRETCFG_b.DTCMPWDSLP = PWRCTRL_MEMRETCFG_DTCMPWDSLP_ALL;
            break;
        case AM_HAL_PWRCTRL_DTCM_8K:
            PWRCTRL->MEMRETCFG_b.DTCMPWDSLP = PWRCTRL_MEMRETCFG_DTCMPWDSLP_ALLBUTGROUP0DTCM0;
            break;
        case AM_HAL_PWRCTRL_DTCM_128K:
            PWRCTRL->MEMRETCFG_b.DTCMPWDSLP = PWRCTRL_MEMRETCFG_DTCMPWDSLP_GROUP1;
            break;
        case AM_HAL_PWRCTRL_DTCM_384K:
            PWRCTRL->MEMRETCFG_b.DTCMPWDSLP = PWRCTRL_MEMRETCFG_DTCMPWDSLP_NONE;
            break;
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_mcu_memory_config()

// ****************************************************************************
//
//  am_hal_pwrctrl_mcu_memory_config_get()
//  Get the MCU Memory configuration.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_memory_config_get(am_hal_pwrctrl_mcu_memory_config_t *psConfig)
{
    //
    // Get the MCU Cache configuration.
    //
    if (PWRCTRL->MEMPWREN_b.PWRENCACHEB0 == PWRCTRL_MEMPWREN_PWRENCACHEB0_EN)
    {
        if (PWRCTRL->MEMPWREN_b.PWRENCACHEB2 == PWRCTRL_MEMPWREN_PWRENCACHEB2_EN)
        {
            psConfig->eCacheCfg = AM_HAL_PWRCTRL_CACHE_ALL;
        }
        else
        {
            psConfig->eCacheCfg = AM_HAL_PWRCTRL_CACHEB0_ONLY;
        }
    }
    else
    {
        if (PWRCTRL->MEMPWREN_b.PWRENCACHEB2 == PWRCTRL_MEMPWREN_PWRENCACHEB2_EN)
        {
            return AM_HAL_STATUS_FAIL;  // Not allowed to select Cache B2 only.
            // This should never be possible.
        }
        else
        {
            psConfig->eCacheCfg = AM_HAL_PWRCTRL_CACHE_NONE;
        }
    }

    //
    // Get the MCU Tightly Coupled Memory configuration.
    //
    psConfig->eDTCMCfg =
        (am_hal_pwrctrl_dtcm_select_e)PWRCTRL->MEMPWREN_b.PWRENDTCM;

    //
    // Get the Non-Volatile Memory configuration.
    //
    psConfig->bEnableNVM0 = PWRCTRL->MEMPWREN_b.PWRENNVM0;

    //
    // Get the Cache retention configuration.
    //
    psConfig->bRetainCache =
        (PWRCTRL->MEMRETCFG_b.CACHEPWDSLP == PWRCTRL_MEMRETCFG_CACHEPWDSLP_DIS);

    //
    // Configure the Non-Volatile Memory retention.
    //
    psConfig->bRetainNVM0 =
        (PWRCTRL->MEMRETCFG_b.NVM0PWDSLP == PWRCTRL_MEMRETCFG_NVM0PWDSLP_DIS);

    //
    // Configure the MCU Tightly Coupled Memory retention.
    //
    if (PWRCTRL->MEMRETCFG_b.DTCMPWDSLP == PWRCTRL_MEMRETCFG_DTCMPWDSLP_ALL)
    {
        psConfig->eRetainDTCM = AM_HAL_PWRCTRL_DTCM_NONE;
    }
    else if (PWRCTRL->MEMRETCFG_b.DTCMPWDSLP == PWRCTRL_MEMRETCFG_DTCMPWDSLP_ALLBUTGROUP0DTCM0)
    {
        psConfig->eRetainDTCM = AM_HAL_PWRCTRL_DTCM_8K;
    }
    else if (PWRCTRL->MEMRETCFG_b.DTCMPWDSLP == PWRCTRL_MEMRETCFG_DTCMPWDSLP_GROUP1)
    {
        psConfig->eRetainDTCM = AM_HAL_PWRCTRL_DTCM_128K;
    }
    else if (PWRCTRL->MEMRETCFG_b.DTCMPWDSLP == PWRCTRL_MEMRETCFG_DTCMPWDSLP_NONE)
    {
        psConfig->eRetainDTCM = AM_HAL_PWRCTRL_DTCM_384K;
    }
    else
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_mcu_memory_config_get()

// ****************************************************************************
//
//  am_hal_pwrctrl_sram_config()
//  Configure the Shared RAM.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_sram_config(am_hal_pwrctrl_sram_memcfg_t *psConfig)
{
    uint32_t      ui32Status;

    //
    // Configure the Shared RAM.
    //
    PWRCTRL->SSRAMPWREN_b.PWRENSSRAM = psConfig->eSRAMCfg;

    DIAG_SUPPRESS_VOLATILE_ORDER()

    //
    // Wait for Status
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              (uint32_t)&PWRCTRL->SSRAMPWRST,
                                              PWRCTRL_SSRAMPWRST_SSRAMPWRST_Msk,
                                              PWRCTRL->SSRAMPWREN,
                                              true);

    //
    // Check for error.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check the Shared RAM power mode status.
    //
    if (PWRCTRL->SSRAMPWRST_b.SSRAMPWRST != PWRCTRL->SSRAMPWREN_b.PWRENSSRAM)
    {
        return AM_HAL_STATUS_FAIL;
    }
    DIAG_DEFAULT_VOLATILE_ORDER()

    //
    // Configure the Shared RAM domain active based on MCU state.
    //
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTMCU = psConfig->eActiveWithMCU;

    //
    // Configure the Shared RAM domain active based on DSP state.
    //
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTDSP = psConfig->eActiveWithDSP;

    //
    // Set the Shared SRAM other active states.
    //
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTGFX  = AM_HAL_PWRCTRL_SRAM_NONE;
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTDISP = AM_HAL_PWRCTRL_SRAM_NONE;


    //
    // Configure the Shared RAM retention.
    //
    switch ( psConfig->eSRAMRetain )
    {
        case   AM_HAL_PWRCTRL_SRAM_NONE:
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_ALL;
            break;
        case AM_HAL_PWRCTRL_SRAM_512K:
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP1;
            break;
        case AM_HAL_PWRCTRL_SRAM_1M:
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_NONE;
            break;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_sram_config()

// ****************************************************************************
//
//  am_hal_pwrctrl_sram_config_get()
//  Get the current Shared RAM configuration.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_sram_config_get(am_hal_pwrctrl_sram_memcfg_t *psConfig)
{
    //
    // Get the Shared RAM configuration.
    //
    psConfig->eSRAMCfg = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMPWREN_b.PWRENSSRAM;

    //
    // Get the SRAM active with MCU configuration.
    //
    psConfig->eActiveWithMCU = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMRETCFG_b.SSRAMACTMCU;

    //
    // Get the SRAM active with DSP configuration.
    //
    psConfig->eActiveWithDSP = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMRETCFG_b.SSRAMACTDSP;

    //
    // Get the SRAM retention configuration.
    //
    if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_ALL)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_NONE;
    }
    else if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_NONE)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_1M;
    }
    else if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP1)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_512K;
    }
    else
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_sram_config_get()

// ****************************************************************************
//
//  am_hal_pwrctrl_dsp_mode_select()
//  Select the DSP power mode.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_dsp_mode_select(am_hal_dsp_select_e eDSP,
                               am_hal_pwrctrl_dsp_mode_e ePowerMode)
{
    uint32_t      ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    // Set the DSP power mode.
    //
    switch ( eDSP )
    {
        case AM_HAL_DSP0:
            PWRCTRL->DSP0PERFREQ_b.DSP0PERFREQ = ePowerMode;
            break;
        case AM_HAL_DSP1:
            PWRCTRL->DSP1PERFREQ_b.DSP1PERFREQ = ePowerMode;
            break;
    }

    //
    // Wait for ACK
    //
    switch ( eDSP )
    {
        case AM_HAL_DSP0:
            ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                                      (uint32_t)&PWRCTRL->DSP0PERFREQ,
                                                      PWRCTRL_DSP0PERFREQ_DSP0PERFACK_Msk,
                                                      (1 << PWRCTRL_DSP0PERFREQ_DSP0PERFACK_Pos),
                                                      true);
        break;
    case AM_HAL_DSP1:
        ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                                  (uint32_t)&PWRCTRL->DSP1PERFREQ,
                                                  PWRCTRL_DSP1PERFREQ_DSP1PERFACK_Msk,
                                                  (1 << PWRCTRL_DSP1PERFREQ_DSP1PERFACK_Pos),
                                                  true);
        break;
    }

    //
    // Check for timeout.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check the DSP power mode status and return SUCCESS/FAIL.
    //
    switch ( eDSP )
    {
        case AM_HAL_DSP0:
            if (ePowerMode != PWRCTRL->DSP0PERFREQ_b.DSP0PERFSTATUS)
            {
                return AM_HAL_STATUS_FAIL;
            }
            break;
        case AM_HAL_DSP1:
            if (ePowerMode != PWRCTRL->DSP1PERFREQ_b.DSP1PERFSTATUS)
            {
                return AM_HAL_STATUS_FAIL;
            }
            break;
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_dsp_mode_select()


// ****************************************************************************
//
//  dsp0_memory_config()
//
// ****************************************************************************
static uint32_t
dsp0_memory_config(am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{
    uint32_t      ui32Status;

    // Configure ICache.
    if (psConfig->bEnableICache)
    {
        PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0ICACHE = PWRCTRL_DSP0MEMPWREN_PWRENDSP0ICACHE_ON;
    }
    else
    {
        PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0ICACHE = PWRCTRL_DSP0MEMPWREN_PWRENDSP0ICACHE_OFF;
    }

    // Configure RAM.
    if (psConfig->bEnableRAM)
    {
        PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0RAM = PWRCTRL_DSP0MEMPWREN_PWRENDSP0RAM_ON;
    }
    else
    {
        PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0RAM = PWRCTRL_DSP0MEMPWREN_PWRENDSP0RAM_OFF;
    }

    DIAG_SUPPRESS_VOLATILE_ORDER()

    //
    // Wait for Status
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              (uint32_t)&PWRCTRL->DSP0MEMPWRST,
                                              AM_HAL_PWRCTRL_DSPMEMPWRST_MASK,
                                              PWRCTRL->DSP0MEMPWREN,
                                              true);

    //
    // Check for error.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check for timeout.
    //
    if ((PWRCTRL->DSP0MEMPWRST_b.PWRSTDSP0ICACHE != PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0ICACHE) ||
        (PWRCTRL->DSP0MEMPWRST_b.PWRSTDSP0RAM != PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0RAM) )
    {
        return AM_HAL_STATUS_FAIL;
    }
    DIAG_DEFAULT_VOLATILE_ORDER()

    // Configure ICache Retention.
    if (psConfig->bRetainCache)
    {
        PWRCTRL->DSP0MEMRETCFG_b.ICACHEPWDDSP0OFF = PWRCTRL_DSP0MEMRETCFG_ICACHEPWDDSP0OFF_RET;
    }
    else
    {
        PWRCTRL->DSP0MEMRETCFG_b.ICACHEPWDDSP0OFF = PWRCTRL_DSP0MEMRETCFG_ICACHEPWDDSP0OFF_PWD;
    }

    // Configure IRAM Retention.
    if (psConfig->bActiveRAM)
    {
        PWRCTRL->DSP0MEMRETCFG_b.DSP0RAMACTMCU = PWRCTRL_DSP0MEMRETCFG_DSP0RAMACTMCU_ACT;
    }
    else
    {
        PWRCTRL->DSP0MEMRETCFG_b.DSP0RAMACTMCU = PWRCTRL_DSP0MEMRETCFG_DSP0RAMACTMCU_WAKEONDEMAND;
    }
    if (psConfig->bRetainRAM)
    {
        PWRCTRL->DSP0MEMRETCFG_b.RAMPWDDSP0OFF = PWRCTRL_DSP0MEMRETCFG_RAMPWDDSP0OFF_RET;
    }
    else
    {
        PWRCTRL->DSP0MEMRETCFG_b.RAMPWDDSP0OFF = PWRCTRL_DSP0MEMRETCFG_RAMPWDDSP0OFF_PWD;
    }

    return AM_HAL_STATUS_SUCCESS;
} // dsp0_memory_config()

// ****************************************************************************
//
//  dsp1_memory_config()
//
// ****************************************************************************
static uint32_t
dsp1_memory_config(am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{
    uint32_t      ui32Status;

    // Configure ICache.
    if (psConfig->bEnableICache)
    {
        PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1ICACHE = PWRCTRL_DSP1MEMPWREN_PWRENDSP1ICACHE_ON;
    }
    else
    {
        PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1ICACHE = PWRCTRL_DSP1MEMPWREN_PWRENDSP1ICACHE_OFF;
    }

    // Configure RAM.
    if (psConfig->bEnableRAM)
    {
        PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1RAM = PWRCTRL_DSP1MEMPWREN_PWRENDSP1RAM_ON;
    }
    else
    {
        PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1RAM = PWRCTRL_DSP1MEMPWREN_PWRENDSP1RAM_OFF;
    }

    DIAG_SUPPRESS_VOLATILE_ORDER()

    //
    // Wait for Status
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              (uint32_t)&PWRCTRL->DSP1MEMPWRST,
                                              AM_HAL_PWRCTRL_DSPMEMPWRST_MASK,
                                              PWRCTRL->DSP1MEMPWREN,
                                              true);

    //
    // Check for error.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check for timeout.
    //
    if ((PWRCTRL->DSP1MEMPWRST_b.PWRSTDSP1ICACHE != PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1ICACHE) ||
        (PWRCTRL->DSP1MEMPWRST_b.PWRSTDSP1RAM != PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1RAM) )
    {
        return AM_HAL_STATUS_FAIL;
    }
    DIAG_DEFAULT_VOLATILE_ORDER()

    // Configure ICache Retention.
    if (psConfig->bRetainCache)
    {
        PWRCTRL->DSP1MEMRETCFG_b.ICACHEPWDDSP1OFF = PWRCTRL_DSP1MEMRETCFG_ICACHEPWDDSP1OFF_RET;
    }
    else
    {
        PWRCTRL->DSP1MEMRETCFG_b.ICACHEPWDDSP1OFF = PWRCTRL_DSP1MEMRETCFG_ICACHEPWDDSP1OFF_PWD;
    }

    // Configure IRAM Retention.
    if (psConfig->bActiveRAM)
    {
        PWRCTRL->DSP1MEMRETCFG_b.DSP1RAMACTMCU = PWRCTRL_DSP1MEMRETCFG_DSP1RAMACTMCU_ACT;
    }
    else
    {
        PWRCTRL->DSP1MEMRETCFG_b.DSP1RAMACTMCU = PWRCTRL_DSP1MEMRETCFG_DSP1RAMACTMCU_WAKEONDEMAND;
    }
    if (psConfig->bRetainRAM)
    {
        PWRCTRL->DSP1MEMRETCFG_b.RAMPWDDSP1OFF = PWRCTRL_DSP1MEMRETCFG_RAMPWDDSP1OFF_RET;
    }
    else
    {
        PWRCTRL->DSP1MEMRETCFG_b.RAMPWDDSP1OFF = PWRCTRL_DSP1MEMRETCFG_RAMPWDDSP1OFF_PWD;
    }

    return AM_HAL_STATUS_SUCCESS;
} // dsp1_memory_config()

// ****************************************************************************
//
//  am_hal_pwrctrl_dsp_memory_config()
//  Configure the DSP memory.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_dsp_memory_config(am_hal_dsp_select_e eDSP,
                                 am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{
    uint32_t    retval = AM_HAL_STATUS_SUCCESS;

    switch ( eDSP )
    {
        case AM_HAL_DSP0:
            retval = dsp0_memory_config(psConfig);
            break;
        case AM_HAL_DSP1:
            retval = dsp1_memory_config(psConfig);
            break;
    }

    return retval;
} // am_hal_pwrctrl_dsp_memory_config()

// ****************************************************************************
//
//  dsp0_memory_get()
//
// ****************************************************************************
static uint32_t
dsp0_memory_get(am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{

    // Read the ICache configuration.
    psConfig->bEnableICache = (PWRCTRL_DSP0MEMPWREN_PWRENDSP0ICACHE_ON == PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0ICACHE );
    psConfig->bRetainCache  = (PWRCTRL_DSP0MEMRETCFG_ICACHEPWDDSP0OFF_RET == PWRCTRL->DSP0MEMRETCFG_b.ICACHEPWDDSP0OFF);

    // Read the RAM configuration.
    psConfig->bEnableRAM    = (PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0RAM == PWRCTRL_DSP0MEMPWREN_PWRENDSP0RAM_ON);
    psConfig->bActiveRAM    = (PWRCTRL->DSP0MEMRETCFG_b.DSP0RAMACTMCU == PWRCTRL_DSP0MEMRETCFG_DSP0RAMACTMCU_ACT);
    psConfig->bRetainRAM    = (PWRCTRL->DSP0MEMRETCFG_b.RAMPWDDSP0OFF == PWRCTRL_DSP0MEMRETCFG_RAMPWDDSP0OFF_RET);

    return AM_HAL_STATUS_SUCCESS;
} // dsp0_memory_get()

// ****************************************************************************
//
//  dsp1_memory_get()
//
// ****************************************************************************
static uint32_t
dsp1_memory_get(am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{
    // Read the ICache configuration.
    psConfig->bEnableICache = (PWRCTRL_DSP1MEMPWREN_PWRENDSP1ICACHE_ON == PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1ICACHE );
    psConfig->bRetainCache  = (PWRCTRL_DSP1MEMRETCFG_ICACHEPWDDSP1OFF_RET == PWRCTRL->DSP1MEMRETCFG_b.ICACHEPWDDSP1OFF);

    // Read the RAM configuration.
    psConfig->bEnableRAM    = (PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1RAM == PWRCTRL_DSP1MEMPWREN_PWRENDSP1RAM_ON);
    psConfig->bActiveRAM    = (PWRCTRL->DSP1MEMRETCFG_b.DSP1RAMACTMCU == PWRCTRL_DSP1MEMRETCFG_DSP1RAMACTMCU_ACT);
    psConfig->bRetainRAM    = (PWRCTRL->DSP1MEMRETCFG_b.RAMPWDDSP1OFF == PWRCTRL_DSP1MEMRETCFG_RAMPWDDSP1OFF_RET);

    return AM_HAL_STATUS_SUCCESS;
} // dsp1_memory_get()

// ****************************************************************************
//
//  am_hal_pwrctrl_dsp_memory_config_get()
//  Get the current the DSP memory configuration.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_dsp_memory_config_get(am_hal_dsp_select_e eDSP,
                                     am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{
    uint32_t      retval = AM_HAL_STATUS_SUCCESS;

    switch ( eDSP )
    {
        case AM_HAL_DSP0:
            retval = dsp0_memory_get(psConfig);
            break;
        case AM_HAL_DSP1:
            retval = dsp1_memory_get(psConfig);
            break;
    }

    return retval;
} // am_hal_pwrctrl_dsp_memory_config_get()

// ****************************************************************************
//
//  am_hal_pwrctrl_periph_enable()
//  Enable power for a peripheral.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_periph_enable(am_hal_pwrctrl_periph_e ePeripheral)
{
    uint32_t      ui32Status;

#ifdef AM_HAL_PWRCTL_CRYPTO_WA
    //
    // Workaround for CRYPTO block power-up issue.
    //
    if (ePeripheral == AM_HAL_PWRCTRL_PERIPH_CRYPTO)
    {
      am_hal_util_write_and_wait((uint32_t*)&PWRCTRL->DEVPWREN,
                                        PWRCTRL_DEVPWREN_PWRENCRYPTO_Msk,
                                        PWRCTRL_DEVPWREN_PWRENCRYPTO_Msk,
                                        AM_HAL_PWRCTL_CRYPTO_DELAY);
    }
    else
#endif // AM_HAL_PWRCTL_CRYPTO_WA
    {
        //
        // Enable power control for the given device.
        //
        AM_CRITICAL_BEGIN
        *(uint32_t *)am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PwrEnRegAddr |=
            am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PeriphEnable;
        AM_CRITICAL_END

      ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                                am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PwrStatReqAddr,
                                                am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PeriphStatus,
                                                am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PeriphStatus,
                                                true);

        //
        // Check for timeout.
        //
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return ui32Status;
        }
    }

    //
    // Check the device status.
    //
    if ( (*(uint32_t *)am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PwrStatReqAddr &
        am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PeriphStatus) > 0)
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        return AM_HAL_STATUS_FAIL;
    }
} // am_hal_pwrctrl_periph_enable()

// ****************************************************************************
//
//  am_hal_pwrctrl_periph_disable()
//  Disable power for a peripheral.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_periph_disable(am_hal_pwrctrl_periph_e ePeripheral)
{
    uint32_t      ui32Status;

#ifdef AM_HAL_PWRCTL_CRYPTO_WA
    //
    // Workaround for CRYPTO block power-up issue.
    //
    if (ePeripheral == AM_HAL_PWRCTRL_PERIPH_CRYPTO)
    {
        am_hal_util_write_and_wait((uint32_t*)&PWRCTRL->DEVPWREN,
                                    PWRCTRL_DEVPWREN_PWRENCRYPTO_Msk,
                                    0,
                                    AM_HAL_PWRCTL_CRYPTO_DELAY);
    }
    else
#endif // AM_HAL_PWRCTL_CRYPTO_WA
    {
        //
        // Disable power domain for the given device.
        //
        AM_CRITICAL_BEGIN
        *(uint32_t *)am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PwrEnRegAddr &=
        ~am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PeriphEnable;
        AM_CRITICAL_END

        ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                                  am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PwrStatReqAddr,
                                                  am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PeriphStatus,
                                                  am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PeriphStatus,
                                                  false);

      //
      // Check for timeout.
      //
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return ui32Status;
        }
    }

    //
    // Check the device status.
    //
    if ( (*(uint32_t *)am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PwrStatReqAddr &
         am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PeriphStatus) == 0)
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        return AM_HAL_STATUS_FAIL;
    }

} // am_hal_pwrctrl_periph_disable()

// ****************************************************************************
//
//  am_hal_pwrctrl_periph_enabled()
//  Determine whether a peripheral is currently enabled.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_periph_enabled(am_hal_pwrctrl_periph_e ePeripheral,
                              bool *bEnabled)
{

    if ( bEnabled == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    *bEnabled = ((*(uint32_t *)am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PwrStatReqAddr &
                  am_hal_pwrctrl_peripheral_control[ePeripheral].ui32PeriphStatus) > 0);

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_periph_enabled()

// ****************************************************************************
//
//  am_hal_pwrctrl_status_get()
//  Get the current powercontrol status registers.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_status_get(am_hal_pwrctrl_status_t *psStatus)
{
    //
    // Device Power ON Status
    //
    psStatus->ui32Device = PWRCTRL->DEVPWRSTATUS;

    //
    // Audio Subsystem ON Status
    //
    psStatus->ui32AudioSS = PWRCTRL->AUDSSPWRSTATUS;

    //
    // MCU Memory Power ON Status
    //
    psStatus->ui32Memory = PWRCTRL->MEMPWRSTATUS;

    //
    // Power ON Status for MCU and DSP0/1 Cores
    //
    psStatus->ui32System = PWRCTRL->SYSPWRSTATUS;

    //
    // Shared SRAM Power ON Status
    //
    psStatus->ui32SSRAM = PWRCTRL->SSRAMPWRST;

    //
    // DSP0 Memories Power ON Status
    //
    psStatus->ui32DSP0MemStatus = PWRCTRL->DSP0MEMPWRST;

    //
    // DSP1 Memories Power ON Status
    //
    psStatus->ui32DSP1MemStatus = PWRCTRL->DSP1MEMPWRST;

    //
    // Voltage Regulators status
    //
    psStatus->ui32VRStatus = PWRCTRL->VRSTATUS;

    //
    // Power Status Register for ADC Block
    //
    psStatus->ui32ADC = PWRCTRL->ADCSTATUS;

    //
    // Power Status Register for audio ADC Block
    //
    psStatus->ui32AudioADC = PWRCTRL->AUDADCSTATUS;

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_status_get()

// ****************************************************************************
//
//  am_hal_pwrctrl_low_power_init()
//  Initialize the device for low power operation.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_low_power_init(void)
{
    uint32_t ui32TrimVer, ui32VDDCtrim;
    bool bDoVDDCLDOboost;

    //
    // Set the default memory configuration.
    //
    am_hal_pwrctrl_mcu_memory_config((am_hal_pwrctrl_mcu_memory_config_t *)&g_DefaultMcuMemCfg);
    am_hal_pwrctrl_sram_config((am_hal_pwrctrl_sram_memcfg_t *)&g_DefaultSRAMCfg);

    //
    // Enable all the clock gating optimizations for Rev B silicon.
    //
    CLKGEN->MISC_b.CLKGENMISCSPARES = 0x3E;  // Do not enable DAXI Clock Gating for now.

    //
    // Set the PWRCTRL PWRWEIGHTS all to 0's.
    //
    PWRCTRL->PWRWEIGHTULP0  = 0;
    PWRCTRL->PWRWEIGHTULP1  = 0;
    PWRCTRL->PWRWEIGHTULP2  = 0;
    PWRCTRL->PWRWEIGHTULP3  = 0;
    PWRCTRL->PWRWEIGHTULP4  = 0;
    PWRCTRL->PWRWEIGHTULP5  = 0;
    PWRCTRL->PWRWEIGHTLP0   = 0;
    PWRCTRL->PWRWEIGHTLP1   = 0;
    PWRCTRL->PWRWEIGHTLP2   = 0;
    PWRCTRL->PWRWEIGHTLP3   = 0;
    PWRCTRL->PWRWEIGHTLP4   = 0;
    PWRCTRL->PWRWEIGHTLP5   = 0;
    PWRCTRL->PWRWEIGHTHP0   = 0;
    PWRCTRL->PWRWEIGHTHP1   = 0;
    PWRCTRL->PWRWEIGHTHP2   = 0;
    PWRCTRL->PWRWEIGHTHP3   = 0;
    PWRCTRL->PWRWEIGHTHP4   = 0;
    PWRCTRL->PWRWEIGHTHP5   = 0;
    PWRCTRL->PWRWEIGHTSLP   = 0;

    //
    //  Set up the Default DAXICFG.
    //
    am_hal_daxi_config(&am_hal_daxi_defaults);
    am_hal_daxi_control(AM_HAL_DAXI_CONTROL_FLUSH, NULL);
    am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, NULL);
    am_hal_delay_us(100);

#if defined(AM_HAL_PWRCTRL_LFRC_SIMOBUCK_TRIM_WA)
    if (!APOLLO4_GE_B2)
    {
      //
      // Set and enable LFRC trims
      //
      MCUCTRL->LFRC_b.TRIMTUNELFRC         = 31;
      MCUCTRL->LFRC_b.LFRCSWE              = MCUCTRL_LFRC_LFRCSWE_OVERRIDE_EN;
      MCUCTRL->LFRC_b.LFRCSIMOCLKDIV       = MCUCTRL_LFRC_LFRCSIMOCLKDIV_DIV2;
    }
#endif

    //
    // If LDO, do some trim updates.
    //
    if ( PWRCTRL->VRSTATUS_b.SIMOBUCKST != PWRCTRL_VRSTATUS_SIMOBUCKST_ACT )
    {
        TrimVersionGet(&ui32TrimVer);
        bDoVDDCLDOboost = ( APOLLO4_GE_B1  &&  (ui32TrimVer >= 6) ) ? true : false;

        if ( bDoVDDCLDOboost )
        {
            if ( !g_bVDDCLDOboosted )
            {
                //
                // Only 1 VDDC boost per purchase please.
                //
                g_bVDDCLDOboosted = true;

                //
                // Get current trim
                //
                ui32VDDCtrim = MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM;

                if ( g_ui32origLDOVDDCtrim == 0xFFFFFFFF )
                {
                    //
                    // Save the original value the very first time.
                    //
                    g_ui32origLDOVDDCtrim = ui32VDDCtrim;
                }

                //
                // Increase VDDC up by 10 steps (~40mv), or max it out.
                //
                ui32VDDCtrim = (ui32VDDCtrim <= (MAX_VDDCTRIM - 10)) ?
                               (ui32VDDCtrim + 10) : MAX_VDDCTRIM;

                AM_CRITICAL_BEGIN
                MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = ui32VDDCtrim;

                //
                // Delay to give voltage supply some time to transition to the new level
                //
                am_hal_delay_us(AM_HAL_PWRCTRL_VDDC_BOOST_DELAY);
                AM_CRITICAL_END
            }
        }
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_low_power_init()

// ****************************************************************************
//
//  am_hal_pwrctrl_control()
//  Additional miscellaneous power controls.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_control(am_hal_pwrctrl_control_e eControl, void *pArgs)
{
    uint32_t ui32ReturnStatus = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32TrimVer, ui32VDDCtrim;
    bool bDoVDDCboost;

    switch ( eControl )
    {
        case AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT:
            //
            // Enable VDDC, VDDF, and VDDS.
            //
            MCUCTRL->SIMOBUCK0 =
                _VAL2FLD(MCUCTRL_SIMOBUCK0_SIMOBUCKVDDSRXCOMPEN, 1) |
                _VAL2FLD(MCUCTRL_SIMOBUCK0_SIMOBUCKVDDFRXCOMPEN, 1) |
                _VAL2FLD(MCUCTRL_SIMOBUCK0_SIMOBUCKVDDCRXCOMPEN, 1);

            //
            // Apollo4 RevB specific power optimizations.
            //
            MCUCTRL->SIMOBUCK15_b.SIMOBUCKZXCOMPOFFSETTRIM = 0;
            MCUCTRL->SIMOBUCK7_b.SIMOBUCKZXCOMPZXTRIM      = 0;

            TrimVersionGet(&ui32TrimVer);
            bDoVDDCboost = ( APOLLO4_GE_B1  && (ui32TrimVer >= 6) ) ? true : false;

            if ( bDoVDDCboost )
            {
                if ( !g_bVDDCbuckboosted )
                {
                    //
                    // Only 1 VDDC boost per purchase please.
                    //
                    g_bVDDCbuckboosted = true;

                    //
                    // Get current trim
                    //
                    ui32VDDCtrim = MCUCTRL->VREFGEN2_b.TVRGVREFTRIM;

                    if ( g_ui32origSimobuckVDDCtrim == 0xFFFFFFFF )
                    {
                        //
                        // Save the original value the very first time.
                        //
                        g_ui32origSimobuckVDDCtrim = ui32VDDCtrim;
                    }

                    //
                    // Increase VDDC up by 24 steps (~40mv), or max it out.
                    //
                    MCUCTRL->SIMOBUCK15_b.SIMOBUCKTRIMLATCHOVER = 1;
                    ui32VDDCtrim = (ui32VDDCtrim <= (MAX_VDDCTRIM - 24)) ?
                                   (ui32VDDCtrim + 24) : MAX_VDDCTRIM;

                    AM_CRITICAL_BEGIN
                    MCUCTRL->VREFGEN2_b.TVRGVREFTRIM = ui32VDDCtrim;

                    //
                    // Delay to give voltage supply some time to transition to the new level
                    //
                    am_hal_delay_us(AM_HAL_PWRCTRL_VDDC_BOOST_DELAY);
                    AM_CRITICAL_END
                }
            }

            //
            // Enable the SIMOBUCK
            //
            PWRCTRL->VRCTRL_b.SIMOBUCKEN = 1;
            break;

#ifdef AM_HAL_PWRCTL_CRYPTO_WA
        case AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN:
            {
                uint32_t    ui32Status;
                bool        bEnabled;

                //
                // Check if CRYPTO block is powered on.
                //
                am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_CRYPTO, &bEnabled);
                if ( bEnabled )
                {
                    //
                    // Power down the crypto block in case it was left on by SBR/SBL.
                    //
                    ui32Status = crypto_powerdown();
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return ui32Status;
                    }
                    ui32Status = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return ui32Status;
                    }
                }
            }
            break;
#endif // AM_HAL_PWRCTL_CRYPTO_WA

        case AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP:
            //
            // This optimization is optional. Enable it IFF the 32KHz crystal is
            // not required during deep sleep. If enabled it will save ~0.8uA.
            //
            MCUCTRL->XTALGENCTRL_b.XTALBIASTRIM   = 0x20;

            MCUCTRL->XTALCTRL =
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALICOMPTRIM,  0 )                                       |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALIBUFTRIM,   0 )                                       |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPPDNB,   MCUCTRL_XTALCTRL_XTALCOMPPDNB_PWRDNCOMP ) |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALPDNB,       MCUCTRL_XTALCTRL_XTALPDNB_PWRDNCORE )     |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPBYPASS, MCUCTRL_XTALCTRL_XTALCOMPBYPASS_USECOMP ) |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOREDISFB,  MCUCTRL_XTALCTRL_XTALCOREDISFB_EN )       |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALSWE,        MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_EN);
            break;

        case AM_HAL_PWRCTRL_CONTROL_DIS_PERIPHS_ALL:
            PWRCTRL->DEVPWREN =
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDBG,     PWRCTRL_DEVPWREN_PWRENDBG_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSBPHY,  PWRCTRL_DEVPWREN_PWRENUSBPHY_DIS)       |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSB,     PWRCTRL_DEVPWREN_PWRENUSB_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENSDIO,    PWRCTRL_DEVPWREN_PWRENSDIO_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENCRYPTO,  PWRCTRL_DEVPWREN_PWRENCRYPTO_DIS)       |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISPPHY, PWRCTRL_DEVPWREN_PWRENDISPPHY_DIS)      |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISP,    PWRCTRL_DEVPWREN_PWRENDISP_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENGFX,     PWRCTRL_DEVPWREN_PWRENGFX_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI2,   PWRCTRL_DEVPWREN_PWRENMSPI2_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI1,   PWRCTRL_DEVPWREN_PWRENMSPI1_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI0,   PWRCTRL_DEVPWREN_PWRENMSPI0_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENADC,     PWRCTRL_DEVPWREN_PWRENADC_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART3,   PWRCTRL_DEVPWREN_PWRENUART3_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART2,   PWRCTRL_DEVPWREN_PWRENUART2_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART1,   PWRCTRL_DEVPWREN_PWRENUART1_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART0,   PWRCTRL_DEVPWREN_PWRENUART0_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM7,    PWRCTRL_DEVPWREN_PWRENIOM7_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM6,    PWRCTRL_DEVPWREN_PWRENIOM6_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM5,    PWRCTRL_DEVPWREN_PWRENIOM5_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM4,    PWRCTRL_DEVPWREN_PWRENIOM4_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM3,    PWRCTRL_DEVPWREN_PWRENIOM3_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM2,    PWRCTRL_DEVPWREN_PWRENIOM2_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM1,    PWRCTRL_DEVPWREN_PWRENIOM1_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM0,    PWRCTRL_DEVPWREN_PWRENIOM0_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOS,     PWRCTRL_DEVPWREN_PWRENIOS_DIS);
            break;

        default:
            ui32ReturnStatus = AM_HAL_STATUS_INVALID_ARG;
            break;
    }

    //
    // Return success status.
    //
    return ui32ReturnStatus;

} // am_hal_pwrctrl_control()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************



