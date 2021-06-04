//*****************************************************************************
//
//! @file nemadc_spi_test.c
//!
//! @brief NemaDC example.
//!
//! This example demonstrates how to drive a SPI4 panel.
//!
//! 4-wire SPI includes 4 signals,
//!   * Chip select (CSX)
//!   * SPI clock (CLK)
//!   * SPI bidirectional data interface (DATA)
//!   * Data and command switch (DCX).
//!
//! During the write sequence the display controller writes one or more bytes of
//! information to the display module via the interface. The write sequence is
//! initiated when CSX is driven from high to low and ends when CSX is pulled high.
//! DCX is driven low while command information is on the interface and is pulled
//! high when data is present.
//!
//! When define TESTMODE_EN to 1 in nemadc_spi_test.c, this example runs at test pattern mode.
//! When define TESTMODE_EN to 0, this example runs at image display mode.
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

// -----------------------------------------------------------------------------
// Copyright (c) 2019 Think Silicon S.A.
// Think Silicon S.A. Confidential Proprietary
// -----------------------------------------------------------------------------
//     All Rights reserved - Unpublished -rights reserved under
//         the Copyright laws of the European Union
//
//  This file includes the Confidential information of Think Silicon S.A.
//  The receiver of this Confidential Information shall not disclose
//  it to any third party and shall protect its confidentiality by
//  using the same degree of care, but not less than a reasonable
//  degree of care, as the receiver uses to protect receiver's own
//  Confidential Information. The entire notice must be reproduced on all
//  authorised copies and copies may only be made to the extent permitted
//  by a licensing agreement from Think Silicon S.A..
//
//  The software is provided 'as is', without warranty of any kind, express or
//  implied, including but not limited to the warranties of merchantability,
//  fitness for a particular purpose and noninfringement. In no event shall
//  Think Silicon S.A. be liable for any claim, damages or other liability, whether
//  in an action of contract, tort or otherwise, arising from, out of or in
//  connection with the software or the use or other dealings in the software.
//
//
//                    Think Silicon S.A.
//                    http://www.think-silicon.com
//                    Patras Science Park
//                    Rion Achaias 26504
//                    Greece
// -----------------------------------------------------------------------------

#include "am_bsp.h"
#include "nema_hal.h"
#include "nema_dc.h"
#include "nema_dc_mipi.h"
#include "am_util_delay.h"
#include "am_util_stdio.h"
#include "nema_dc_regs.h"
#include "oli_200x200_rgba.h"
#include "tsi_malloc.h"
#include "string.h"
#include "am_devices_nemadc_rm67162.h"

#define TESTMODE_EN 1
#if TESTMODE_EN
    #define FB_RESX 390
    #define FB_RESY 390
#else
    #define FB_RESX 200
    #define FB_RESY 200
#endif
//*****************************************************************************
//
//! @brief Test SPI4 interface
//!
//! @param i32PixelFormat Panel pixel format.
//!
//! @return None.
//
//*****************************************************************************
void
test_MIPI_SPI(int32_t i32PixelFormat)
{
    uint16_t ui16PanelResX = g_sDispCfg[g_eDispType].ui32PanelResX; //!< panel's max resolution
    uint16_t ui16PanelResY = g_sDispCfg[g_eDispType].ui32PanelResY; //!< panel's max resolution
    uint16_t ui16MinX, ui16MinY;
    //
    // Set the display region to center of panel
    //
    if (FB_RESX > ui16PanelResX)

    {
        ui16MinX = 0;   //!< set the minimum value to 0
    }
    else
    {
        ui16MinX = (ui16PanelResX - FB_RESX) >> 1;
        ui16MinX = (ui16MinX >> 1) << 1;
    }

    if (FB_RESY > ui16PanelResY)
    {
        ui16MinY = 0;   //!< set the minimum value to 0
    }
    else
    {
        ui16MinY = (ui16PanelResY - FB_RESY) >> 1;
        ui16MinY = (ui16MinY >> 1) << 1;
    }

    am_devices_nemadc_rm67162_init(MIPICFG_SPI4, i32PixelFormat, FB_RESX, FB_RESY, ui16MinX, ui16MinY);

#if TESTMODE_EN
    //
    // Start MIPI Panel Memory Write
    //
    nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_write_memory_start);

    nemadc_layer_enable(0);

    nemadc_reg_write(NEMADC_REG_INTERRUPT, 1 << 4); //!< Enable frame end interrupt
    //
    // Send One Frame
    //
    nemadc_set_mode(NEMADC_ONE_FRAME | NEMADC_TESTMODE);
    //
    // Wait for transfer to be completed
    //
    nemadc_wait_vsync();
#else
    //
    // send layer 0 to display via NemaDC
    //
    nemadc_layer_t sLayer0;
    sLayer0.resx          = FB_RESX;
    sLayer0.resy          = FB_RESY;
    sLayer0.buscfg        = 0;
    sLayer0.format        = NEMADC_RGBA8888;
    sLayer0.blendmode     = NEMADC_BL_SRC;
    sLayer0.stride        = sLayer0.resx * 4;
    sLayer0.startx        = 0;
    sLayer0.starty        = 0;
    sLayer0.sizex         = sLayer0.resx;
    sLayer0.sizey         = sLayer0.resy;
    sLayer0.alpha         = 0xff;
    sLayer0.baseaddr_virt = tsi_malloc(sLayer0.resy * sLayer0.stride);
    sLayer0.baseaddr_phys = (unsigned)(sLayer0.baseaddr_virt);

    memcpy((char*)sLayer0.baseaddr_virt, ui8Oli200x200RGBA, sizeof(ui8Oli200x200RGBA));

    // Program NemaDC Layer0
    nemadc_set_layer(0, &sLayer0); //This function includes layer enable.

    nemadc_send_frame_single();
    tsi_free(sLayer0.baseaddr_virt);
#endif
}

int nemadc_spi_test()
{
    nema_sys_init();
    //
    // Initialize NemaDC
    //
    if (nemadc_init() != 0)
    {
        return -2;
    }
    if (g_sDispCfg[g_eDispType].bUseDPHYPLL == true)
    {
        uint8_t ui8LanesNum = g_sDsiCfg.ui8NumLanes;
        uint8_t ui8DbiWidth = g_sDsiCfg.eDbiWidth;
        uint32_t ui32FreqTrim = g_sDsiCfg.eDsiFreq;
        if (am_hal_dsi_para_config(ui8LanesNum, ui8DbiWidth, ui32FreqTrim) != 0)
        {
            return -3;
        }
    }

    test_MIPI_SPI(MIPICFG_1RGB888_OPT0);
    am_util_delay_ms(1000);
    test_MIPI_SPI(MIPICFG_1RGB666_OPT0);
    am_util_delay_ms(1000);
    test_MIPI_SPI(MIPICFG_1RGB565_OPT0);
    am_util_delay_ms(1000);
    test_MIPI_SPI(MIPICFG_1RGB332_OPT0);

    return 0;
}
