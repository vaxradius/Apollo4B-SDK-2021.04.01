//*****************************************************************************
//
//! @file nemagfx_stress_test.c
//!
//! @brief NemaGFX example.
//! This example uses MSPI, IOM, DMA, GPU, and DC to read/write data from different
//! ram sections to put enormous pressure on the AXI bus.
//! The GUI task also demonstrates how to use the partial frame buffer and ping-pong
//! buffer features in a pipeline workflow for DMA, GPU, and DC. In GUI task,
//! DMA copy texture from external PSRAM to internal SSRAM or ExtendedRAM,
//! GPU render these textures to frame buffer, and DC transfer the framebuffer to
//! display panel, these three steps are organized in a pipeline to make DMA, GPU,
//! and DC work in parallel.
//!
//! SMALLFB
//! undefine this to disable partial framebuffer features
//! SMALLFB_STRIPES
//! controls how many stripes to divide the whole framebuffer
//!
//! Note: This example needs PSRAM devices connected to MSPI0, if you encounter
//! hardfault, please check your PSRAM setting.You are supposed to see a digital
//! Quartz clock if GUI task runs successfully.
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

//*****************************************************************************
//
// This application has a large number of common include files. For
// convenience, we'll collect them all together in a single header and include
// that everywhere.
//
//*****************************************************************************
#include "nemagfx_stress_test.h"

#include "am_bsp.h"

#include "nema_core.h"
#include "nema_utils.h"
#include "nema_event.h"
#include "nema_dc.h"
#include "nema_hal.h"
#include "nema_dc_mipi.h"

#include "am_devices_dsi_rm67162.h"
#include "am_devices_nemadc_rm67162.h"
#include "am_devices_mspi_psram_aps12808l.h"

#include "w0_360x360_rgba565.h"
#include "w0_360x196_rgba4444.h"
#include "w1_28x125_rgba.h"
#include "w2_28x174_rgba.h"
#include "w3_21x182_rgba.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define SMALLFB
#define SMALLFB_STRIPES (4)

#define DC_LAYER_TO_USE (0)

//Address of the textures in PSRAM
#define PSRAM_W3_21x182_RGBA       (MSPI_XIP_BASE_ADDRESS)
#define PSRAM_W2_28x174_RGBA       (((PSRAM_W3_21x182_RGBA + w3_21x182_rgba_length + 8) >> 3) << 3)
#define PSRAM_W1_28x125_RGBA       (((PSRAM_W2_28x174_RGBA + w2_28x174_rgba_length + 8) >> 3) << 3)
#define PSRAM_W0_360x360_RGBA565   (((PSRAM_W1_28x125_RGBA + w1_28x125_rgba_length + 8) >> 3) << 3)
#define PSRAM_W0_360x196_RGBA4444  (((PSRAM_W0_360x360_RGBA565 + w0_360x360_rgba565_length + 8) >> 3) << 3)

//Address of the textures in PSRAM without XIP mapping
#define PSRAM_W3_21x182_RGBA_UNMAPED       (0)
#define PSRAM_W2_28x174_RGBA_UNMAPED       (((PSRAM_W3_21x182_RGBA_UNMAPED + w3_21x182_rgba_length + 8) >> 3) << 3)
#define PSRAM_W1_28x125_RGBA_UNMAPED       (((PSRAM_W2_28x174_RGBA_UNMAPED + w2_28x174_rgba_length + 8) >> 3) << 3)
#define PSRAM_W0_360x360_RGBA565_UNMAPED   (((PSRAM_W1_28x125_RGBA_UNMAPED + w1_28x125_rgba_length + 8) >> 3) << 3)
#define PSRAM_W0_360x196_RGBA4444_UNMAPED  (((PSRAM_W0_360x360_RGBA565_UNMAPED + w0_360x360_rgba565_length + 8) >> 3) << 3)

//clock hands location
#define SSEC_PIVOT_X (-10)
#define SSEC_PIVOT_Y (-158)

#define MIN_PIVOT_X  (-14)
#define MIN_PIVOT_Y  (-157)

#define HOUR_PIVOT_X (-13)
#define HOUR_PIVOT_Y (-109)

//Frame buffer size
#define FB_RESX (360U)
#define FB_RESY (360U)

//Offset of the panel
#define PANEL_OFFSET_X ((g_sDispCfg[g_eDispType].ui32PanelResX-FB_RESX)/2)
#define PANEL_OFFSET_Y ((g_sDispCfg[g_eDispType].ui32PanelResY-FB_RESY)/2)

#ifdef SMALLFB
#ifndef SMALLFB_STRIPES
#define SMALLFB_STRIPES (4)
#endif
#else
#undef SMALLFB_STRIPES
#define SMALLFB_STRIPES (1)
#endif

//DSP ram region
#define DSP0_IRAM_ADDRESS (0x10160000)
#define DSP0_DRAM_ADDRESS (0x10180000)
#define DSP1_IRAM_ADDRESS (0x101C0000)
#define DSP1_DRAM_ADDRESS (0x101D0000)

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
//layer
nemadc_layer_t g_sLayer =
{
    .startx        = 0,
    .sizex         = FB_RESX,
    .resx          = FB_RESX,
    .starty        = 0,
    .sizey         = FB_RESY / SMALLFB_STRIPES,
    .resy          = FB_RESY / SMALLFB_STRIPES,
    .stride        = -1,
    .format        = NEMADC_RGBA8888,
    .blendmode     = NEMADC_BL_SRC,
    .buscfg        = 0,
    .alpha         = 0xff,
};

//Textures
static img_obj_t g_sHourHand =       {{0},  28, 125, -1, 1, NEMA_RGBA8888, NEMA_FILTER_BL};
static img_obj_t g_sMinHand =        {{0},  28, 174, -1, 1, NEMA_RGBA8888, NEMA_FILTER_BL};
static img_obj_t g_sSecHand =       {{0},  21, 182, -1, 1, NEMA_RGBA8888, NEMA_FILTER_BL};
//static img_obj_t g_sClockDial =   {{0},  360, 360, -1, 1, NEMA_RGB565, NEMA_FILTER_BL};
//static img_obj_t g_sSportsDial =  {{0},  360, 196, -1, 1, NEMA_RGBA4444, NEMA_FILTER_BL};

//Frame buffer
static img_obj_t g_sFrameBuffer[2] =
{
    {{0},  FB_RESX, FB_RESY / SMALLFB_STRIPES, -1, 1, NEMA_RGBA8888, NEMA_FILTER_BL},
    {{0},  FB_RESX, FB_RESY / SMALLFB_STRIPES, -1, 1, NEMA_RGBA8888, NEMA_FILTER_BL},
};

static img_obj_t g_sClockDialBuffer[2] =
{
    {{0},  FB_RESX, FB_RESY / SMALLFB_STRIPES, -1, 1, NEMA_RGB565, NEMA_FILTER_BL},
    {{0},  FB_RESX, FB_RESY / SMALLFB_STRIPES, -1, 1, NEMA_RGB565, NEMA_FILTER_BL},
};

static img_obj_t g_sSportsDialBuffer[2] =
{
    {{0},  FB_RESX, FB_RESY / SMALLFB_STRIPES, -1, 1, NEMA_RGBA4444, NEMA_FILTER_BL},
    {{0},  FB_RESX, FB_RESY / SMALLFB_STRIPES, -1, 1, NEMA_RGBA4444, NEMA_FILTER_BL},
};

//Pointer to frame buffer
static img_obj_t* g_pFrameBufferGPU = &g_sFrameBuffer[0];
static img_obj_t* g_pFrameBufferDC = &g_sFrameBuffer[1];

//Pointer to texture buffer
static img_obj_t* g_pClockDialBufferDMA = &g_sClockDialBuffer[0];
static img_obj_t* g_pClockDialBufferGPU = &g_sClockDialBuffer[1];

static img_obj_t* g_pSportsDialBufferDMA = &g_sSportsDialBuffer[0];
static img_obj_t* g_pSportsDialBufferGPU = &g_sSportsDialBuffer[1];

//flag to sync DMA finish
#ifndef BAREMETAL
static SemaphoreHandle_t g_semDMAFinish = NULL;
#else
static bool g_bDMAFinish = false;
#endif

//*****************************************************************************
//
// External variable definitions
//
//*****************************************************************************
extern void *g_pPsramHandle;
extern void *g_pMSPIPsramHandle;


//*****************************************************************************
//
// Allocate buffer in SSRAM using tsi_malloc
//
//*****************************************************************************
int
SSRAM_buffer_alloc(img_obj_t* img)
{
    uint32_t size;

    size = nema_texture_size(img->format, 0, img->w, img->h);

    img->bo = nema_buffer_create(size);

    if ( img->bo.base_virt == NULL )
    {
      am_util_stdio_printf("TSI buffer Malloc failed!\n");
      return -1;
    }

    return 0;
}

//*****************************************************************************
//
// Init texture
//
//*****************************************************************************
int
texture_framebuffer_init(void)
{
    int ret;

    // load textures to PSRAM
    memcpy((void*)PSRAM_W3_21x182_RGBA,      w3_21x182_rgba,      w3_21x182_rgba_length);
    memcpy((void*)PSRAM_W2_28x174_RGBA,      w2_28x174_rgba,      w2_28x174_rgba_length);
    memcpy((void*)PSRAM_W1_28x125_RGBA,      w1_28x125_rgba,      w1_28x125_rgba_length);
    memcpy((void*)PSRAM_W0_360x360_RGBA565,  w0_360x360_rgba565,  w0_360x360_rgba565_length);
    memcpy((void*)PSRAM_W0_360x196_RGBA4444, w0_360x196_rgba4444, w0_360x196_rgba4444_length);

    // alloc textures buffer in psram
    g_sSecHand.bo.base_phys = PSRAM_W3_21x182_RGBA;
    g_sSecHand.bo.base_virt = (void*)PSRAM_W3_21x182_RGBA;
    g_sSecHand.bo.size = w3_21x182_rgba_length;

    g_sMinHand.bo.base_phys = PSRAM_W2_28x174_RGBA;
    g_sMinHand.bo.base_virt = (void*)PSRAM_W2_28x174_RGBA;
    g_sMinHand.bo.size = w2_28x174_rgba_length;

    g_sHourHand.bo.base_phys = PSRAM_W1_28x125_RGBA;
    g_sHourHand.bo.base_virt = (void*)PSRAM_W1_28x125_RGBA;
    g_sHourHand.bo.size = w1_28x125_rgba_length;

//    g_sClockDial.bo.base_phys = PSRAM_W0_360x360_RGBA565;
//    g_sClockDial.bo.base_virt = (void*)PSRAM_W0_360x360_RGBA565;
//    g_sClockDial.bo.size = w0_360x360_rgba565_length;
//
//    g_sSportsDial.bo.base_phys = PSRAM_W0_360x196_RGBA4444;
//    g_sSportsDial.bo.base_virt = (void*)PSRAM_W0_360x196_RGBA4444;
//    g_sSportsDial.bo.size = w0_360x196_rgba4444_length;

    // alloc textures buffer in extended_ram
    g_sClockDialBuffer[0].bo.base_phys =  DSP0_IRAM_ADDRESS;
    g_sClockDialBuffer[0].bo.base_virt =  (void*)g_sClockDialBuffer[0].bo.base_phys;
    g_sClockDialBuffer[0].bo.size = nema_texture_size(g_sClockDialBuffer[0].format,
                                                    0,
                                                    g_sClockDialBuffer[0].w,
                                                    g_sClockDialBuffer[0].h);

    g_sSportsDialBuffer[1].bo.base_phys  = DSP1_IRAM_ADDRESS;
    g_sSportsDialBuffer[1].bo.base_virt  = (void*)g_sSportsDialBuffer[1].bo.base_phys;
    g_sSportsDialBuffer[1].bo.size = nema_texture_size(g_sSportsDialBuffer[1].format,
                                                    0,
                                                    g_sSportsDialBuffer[1].w,
                                                    g_sSportsDialBuffer[1].h);

    // alloc textures buffer in SSRAM
    ret = SSRAM_buffer_alloc(&g_sClockDialBuffer[1]);
    if ( ret < 0 )
    {
        return ret;
    }

    ret = SSRAM_buffer_alloc(&g_sSportsDialBuffer[0]);
    if ( ret < 0 )
    {
        nema_buffer_destroy(&g_sClockDialBuffer[1].bo);
        return ret;
    }

    // alloc framebuffer space in SSRAM
    ret = SSRAM_buffer_alloc(&g_sFrameBuffer[0]);
    if ( ret < 0 )
    {
        nema_buffer_destroy(&g_sClockDialBuffer[1].bo);
        nema_buffer_destroy(&g_sFrameBuffer[0].bo);
        return ret;
    }

    // alloc frame buffer in extended_ram.
    g_sFrameBuffer[1].bo.base_phys =  DSP0_DRAM_ADDRESS;
    g_sFrameBuffer[1].bo.base_virt =  (void*)g_sFrameBuffer[1].bo.base_phys;
    g_sFrameBuffer[1].bo.size = nema_texture_size(g_sFrameBuffer[1].format, 0, g_sFrameBuffer[1].w, g_sFrameBuffer[1].h);

    return 0;
}


//*****************************************************************************
//
// Swap framebuffer
//
//*****************************************************************************
void
frameBuffer_swap(void)
{
    img_obj_t* temp;

    temp = g_pFrameBufferGPU;
    g_pFrameBufferGPU = g_pFrameBufferDC;
    g_pFrameBufferDC = temp;
}

//*****************************************************************************
//
// Swap texture buffer
//
//*****************************************************************************
void
textureBuffer_swap(void)
{
    img_obj_t* temp;

    temp = g_pSportsDialBufferDMA;
    g_pSportsDialBufferDMA = g_pSportsDialBufferGPU;
    g_pSportsDialBufferGPU = temp;

    temp = g_pClockDialBufferDMA;
    g_pClockDialBufferDMA = g_pClockDialBufferGPU;
    g_pClockDialBufferGPU = temp;
}

//*****************************************************************************
//
// MSPI DMA finish callback
//
//*****************************************************************************
void
MspiTransferCallback(void *pCallbackCtxt, uint32_t status)
{
#ifdef BAREMETAL
    g_bDMAFinish = true;
#else
    xSemaphoreGiveFromISR(g_semDMAFinish, NULL);
#endif
}

//*****************************************************************************
//
// Load texture by DMA
//
//*****************************************************************************
void
clock_dial_texture_load(uint32_t parts)
{
    uint32_t offset;
    uint32_t length;

    offset = g_sClockDialBuffer[0].bo.size * parts;
    length = g_sClockDialBuffer[0].bo.size;

#ifdef BAREMETAL
    g_bDMAFinish = false;
#endif

    am_devices_mspi_psram_aps12808l_ddr_nonblocking_read(g_pPsramHandle,
                                                         g_pClockDialBufferDMA->bo.base_virt,
                                                         PSRAM_W0_360x360_RGBA565_UNMAPED + offset,
                                                         length,
                                                         MspiTransferCallback,
                                                         NULL);
}

//*****************************************************************************
//
// Load texture by CPU memory copy
//
//*****************************************************************************
void
sports_dial_texture_load(uint32_t parts)
{
    uint32_t offset;
    int32_t length;
    int32_t left;
    int32_t stride;

    offset = g_sSportsDialBuffer[0].bo.size * parts;
    left = w0_360x196_rgba4444_length - offset;

    if ( left < 0 )
    {
        left = 0;
    }

    length = (left < g_sSportsDialBuffer[0].bo.size) ? left : g_sSportsDialBuffer[0].bo.size;

    memcpy(g_pSportsDialBufferDMA->bo.base_virt, (void*)(PSRAM_W0_360x196_RGBA4444 + offset), length);

    stride = nema_stride_size(g_sSportsDialBuffer[0].format, 0, g_sSportsDialBuffer[0].w);

    g_pSportsDialBufferDMA->h = length / stride;

}

//*****************************************************************************
//
// Set up GP, DC and panel.
//
//*****************************************************************************
int
GPU_DC_setup(void)
{
    int ret;
    uint16_t ui16PanelResX = g_sDispCfg[g_eDispType].ui32PanelResX; //panel's max resolution
    uint16_t ui16PanelResY = g_sDispCfg[g_eDispType].ui32PanelResY; //panel's max resolution
    uint32_t ui32MipiCfg = MIPICFG_8RGB888_OPT0; // default config
    uint16_t minx, miny;

    //Initialize NemaGFX
    ret = nema_init();
    if (ret != 0)
    {
        am_util_stdio_printf("GPU init failed!\n");
        return ret;
    }
    //Initialize Nema|dc
    ret = nemadc_init();
    if (ret != 0)
    {
        am_util_stdio_printf("DC init failed!\n");
        return ret;
    }

    if ((g_sDispCfg[g_eDispType].eInterface == IF_DSI) || (g_sDispCfg[g_eDispType].bUseDPHYPLL == true))
    {
        uint8_t ui8LanesNum = g_sDsiCfg.ui8NumLanes;
        uint8_t ui8DbiWidth = g_sDsiCfg.eDbiWidth;
        uint32_t ui32FreqTrim = g_sDsiCfg.eDsiFreq;
        pixel_format_t eFormat = FMT_RGB888;
        if ( am_hal_dsi_para_config(ui8LanesNum, ui8DbiWidth, ui32FreqTrim) != 0 )
        {
            return -3;
        }
        switch ( eFormat )
        {
            case FMT_RGB888:
                if ( ui8DbiWidth == 16 )
                {
                    ui32MipiCfg = MIPICFG_16RGB888_OPT0;
                }
                if ( ui8DbiWidth ==  8 )
                {
                    ui32MipiCfg = MIPICFG_8RGB888_OPT0;
                }
                break;

            case FMT_RGB565:
                if ( ui8DbiWidth == 16 )
                {
                    ui32MipiCfg = MIPICFG_16RGB565_OPT0;
                }
                if ( ui8DbiWidth ==  8 )
                {
                    ui32MipiCfg = MIPICFG_8RGB565_OPT0;
                }
                break;

            default:
                //
                // invalid color component index
                //
                return -3;
        }
    }

    //Set the display region to center
    if ( FB_RESX > ui16PanelResX )
    {
        minx = 0; // set the minimum value to 0
    }
    else
    {
        minx = (ui16PanelResX - FB_RESX) >> 1;
        minx = (minx >> 1) << 1;
    }

    if ( FB_RESY > ui16PanelResY )
    {
        miny = 0; // set the minimum value to 0
    }
    else
    {
        miny = (ui16PanelResY - FB_RESY) >> 1;
        miny = (miny >> 1) << 1;
    }

    //Initialize the display
    switch (g_sDispCfg[g_eDispType].eInterface)
    {
        case IF_SPI4:
            am_devices_nemadc_rm67162_init(MIPICFG_SPI4, MIPICFG_1RGB565_OPT0, FB_RESX, FB_RESY / SMALLFB_STRIPES, minx, miny);
            break;
        case IF_DSPI:
            am_devices_nemadc_rm67162_init(MIPICFG_DSPI | MIPICFG_SPI4, MIPICFG_2RGB888_OPT0, FB_RESX, FB_RESY / SMALLFB_STRIPES, minx, miny);
            break;
        case IF_QSPI:
            am_devices_nemadc_rm67162_init(MIPICFG_QSPI | MIPICFG_SPI4, MIPICFG_4RGB565_OPT0, FB_RESX, FB_RESY / SMALLFB_STRIPES, minx, miny);
            break;
        case IF_DSI:
            am_devices_dsi_rm67162_init(ui32MipiCfg, FB_RESX, FB_RESY / SMALLFB_STRIPES, minx, miny);
            break;
        default:
            ; //NOP
    }
    return 0;
}

//*****************************************************************************
//
// Draw needle
//
//*****************************************************************************
static inline void
needle_draw( img_obj_t *img, float x0, float y0, float angle, int cx, int cy )
{
    float x1 = x0 + img->w,  y1 = y0;
    float x2 = x0 + img->w,  y2 = y0 + img->h;
    float x3 = x0       ,  y3 = y0 + img->h;

    //calculate rotation matrix
    nema_matrix3x3_t m;
    nema_mat3x3_load_identity(m);
    nema_mat3x3_rotate(m, -angle);
    nema_mat3x3_translate(m, cx, cy);

    //rotate points
    nema_mat3x3_mul_vec(m, &x0, &y0);
    nema_mat3x3_mul_vec(m, &x1, &y1);
    nema_mat3x3_mul_vec(m, &x2, &y2);
    nema_mat3x3_mul_vec(m, &x3, &y3);

    //draw needle
    nema_bind_src_tex(img->bo.base_phys, img->w, img->h, img->format, img->stride, NEMA_FILTER_BL);
    nema_blit_quad_fit(x0, y0,
                       x1, y1,
                       x2, y2,
                       x3, y3);
}

//*****************************************************************************
//
// Draw watchface
//
//*****************************************************************************
int
watchface_draw(nema_cmdlist_t *cl, float hour, float min, float sec, int parts)
{
    float angle;

    //rewind and bind the CL
    nema_cl_rewind(cl);
    nema_cl_bind(cl);

    //bind the destination buffer
    nema_bind_dst_tex(g_pFrameBufferGPU->bo.base_phys, FB_RESX, FB_RESY, g_pFrameBufferGPU->format, g_pFrameBufferGPU->stride);

    //draw backface
    nema_set_clip(0, 0, FB_RESX, FB_RESY / SMALLFB_STRIPES);

    nema_bind_src_tex(g_pClockDialBufferGPU->bo.base_phys,
                      g_pClockDialBufferGPU->w,
                      g_pClockDialBufferGPU->h,
                      g_pClockDialBufferGPU->format,
                      g_pClockDialBufferGPU->stride,
                      0);

    nema_set_blend_blit(NEMA_BLOP_MODULATE_A | NEMA_BL_SIMPLE);
    nema_blit(0, 0);

    nema_bind_src_tex(g_pSportsDialBufferGPU->bo.base_phys,
                  g_pSportsDialBufferGPU->w,
                  g_pSportsDialBufferGPU->h,
                  g_pSportsDialBufferGPU->format,
                  g_pSportsDialBufferGPU->stride,
                  0);
    nema_blit(0, 0);

    nema_set_blend_blit(NEMA_BL_SIMPLE);

    //draw needle
    angle =  hour / 12.f * 360.f;
    needle_draw(&g_sHourHand, HOUR_PIVOT_X, HOUR_PIVOT_Y, -angle, FB_RESX / 2, FB_RESY / 2 - FB_RESY / SMALLFB_STRIPES * parts);

    angle =  min / 60.f * 360.f;
    needle_draw(&g_sMinHand, MIN_PIVOT_X, MIN_PIVOT_Y, -angle, FB_RESX / 2, FB_RESY / 2 - FB_RESY / SMALLFB_STRIPES * parts);

    angle = sec / 60.f * 360.f;
    needle_draw(&g_sSecHand, SSEC_PIVOT_X, SSEC_PIVOT_Y, -angle, FB_RESX / 2, FB_RESY / 2 - FB_RESY / SMALLFB_STRIPES * parts);

    return 0;
}

//*****************************************************************************
//
// Set pannel display region
//
//*****************************************************************************
#ifdef SMALLFB
void
display_region_set(int8_t parts)
{
    if (g_sDispCfg[g_eDispType].eInterface == IF_DSI)
    {
        am_devices_dsi_rm67162_set_region(FB_RESX, FB_RESY / SMALLFB_STRIPES, PANEL_OFFSET_X, PANEL_OFFSET_Y + (FB_RESY / SMALLFB_STRIPES) * parts);
    }
    else if (g_sDispCfg[g_eDispType].eInterface == IF_QSPI)
    {
        am_devices_nemadc_rm67162_set_region(MIPICFG_QSPI, FB_RESX, FB_RESY / SMALLFB_STRIPES, PANEL_OFFSET_X, PANEL_OFFSET_Y + (FB_RESY / SMALLFB_STRIPES) * parts);
    }
    else if (g_sDispCfg[g_eDispType].eInterface == IF_SPI4)
    {
        am_devices_nemadc_rm67162_set_region(MIPICFG_SPI4, FB_RESX, FB_RESY / SMALLFB_STRIPES, PANEL_OFFSET_X, PANEL_OFFSET_Y + (FB_RESY / SMALLFB_STRIPES) * parts);
    }
    else if (g_sDispCfg[g_eDispType].eInterface == IF_DSPI)
    {
        am_devices_nemadc_rm67162_set_region(MIPICFG_DSPI, FB_RESX, FB_RESY / SMALLFB_STRIPES, PANEL_OFFSET_X, PANEL_OFFSET_Y + (FB_RESY / SMALLFB_STRIPES) * parts);
    }
}
#endif

//*****************************************************************************
//
// main loop
//
//*****************************************************************************
int
watchface(void)
{
    int ret;
    static nema_cmdlist_t cl;
    float time = 0;
    int8_t parts = 0;
    int8_t texture_part = -1;
    int8_t GPU_part = -2;
    int8_t DC_part = -3;

    // Set up GPU, DC, display panel
    GPU_DC_setup();

    // Allocate memory for framebuffer and textures
    ret = texture_framebuffer_init();
    if ( ret < 0 )
    {
        return ret;
    }

    // Create GPU command list
    cl = nema_cl_create_sized(0x1000);

    // Set DC layer to use.
    nemadc_set_layer(DC_LAYER_TO_USE, &g_sLayer);

#ifndef BAREMETAL
    g_semDMAFinish = xSemaphoreCreateBinary();
    if ( g_semDMAFinish == NULL )
    {
        am_util_stdio_printf("Create semphone failed!\n");
        return -1;
    }
#endif

    while(1)
    {

        //Get time from timer
        time = nema_get_time();

        float sec  = time;
        float min  = time / 60;
        float hour = time / 60 / 60;

#ifdef SMALLFB
        for (parts = 0; parts < SMALLFB_STRIPES; parts++)
#endif
        {

            //Step the pipeline
            DC_part = GPU_part;
            GPU_part = texture_part;
            texture_part = parts;

            //DC workflow
            if ( DC_part >= 0 )
            {
                //Set panel refresh region
                #ifdef SMALLFB
                display_region_set(DC_part);
                #endif

                //Set the data location to be send by DC.
                nemadc_set_layer_addr(DC_LAYER_TO_USE, g_pFrameBufferDC->bo.base_phys);

                //Start DC
                dsi_send_frame_single_start(NEMADC_OUTP_OFF);
            }

            //GPU workflow
            if ( GPU_part >= 0 )
            {
                //draw watchface
                watchface_draw(&cl, hour, min, sec, GPU_part);

                //start GPU, submit CL
                nema_cl_submit(&cl);
            }

            //DMA workflow
            if ( texture_part >= 0 )
            {
                //Start DMA to load texture to texture buffer
                clock_dial_texture_load(texture_part);

                //Copy texture 2, it's a block operation, so we start it after DMA, GPU and DC are triggered.
                sports_dial_texture_load(texture_part);
            }

            //wait DC
            if ( DC_part >= 0 )
            {
                //Wait DC complete interrupt.
                nemadc_wait_vsync();
                //Do follow-up operations required by hardware.
                dsi_send_frame_single_end();
            }

            //wait GPU
            if ( GPU_part >= 0 )
            {
                nema_cl_wait(&cl);
            }

            //wait DMA
            if ( texture_part >= 0 )
            {
#ifdef BAREMETAL
                while(g_bDMAFinish == false);
#else
                xSemaphoreTake( g_semDMAFinish, portMAX_DELAY);
#endif
            }

            //swap texture buffer
            if ( texture_part >= 0 )
            {
                textureBuffer_swap();
            }

            //swap frame buffer
            if ( GPU_part >= 0 )
            {
                frameBuffer_swap();
            }

        }

        //print fps
        nema_calculate_fps();
    }

}



