//*****************************************************************************
//
//! @file nemagfx_performance_test.c
//!
//! @brief NemaGFX performace test example.
//!
//! This example put texture or framebuffer at PSRAM and run various baseline
//! test of GPU. The following operations are included:copy, blend, scale,
//! rotate, scale+rotate.
//!
//! HOW TO USE: Compile->Download->Collect SWO output->Copy results to a file
//! ->rename this file as *.csv -> open by Excel -> save it as *.xlsx
//!
//! Note: Make sure the PSRAM is connected before running this test.
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

#include "am_bsp.h"
#include "am_util.h"

#include "nema_core.h"
#include "nema_utils.h"
#include "nema_event.h"
#include "nema_dc.h"
#include "nema_dc_mipi.h"

#include "am_devices_dsi_rm67162.h"
#include "am_devices_nemadc_rm67162.h"

#include "main.h"

//
// Textures
//
//#include "oli_390x390_rgba5551.h"
//#include "oli_100x100_rgba5551.h"
#include "oli_100x100_argb8888.h"
#include "oli_100x100_rgba565.h"
#include "oli_100x100_tsc6.h"
#include "oli_360x360_argb8888.h"
#include "oli_360x360_rgba565.h"
#include "oli_360x360_tsc6.h"
#include "simhei18pt4b.h"

//#define psram_oli_100x100_RGBA5551              MSPI_XIP_BASE_ADDRESS
//#define psram_oli_390x390_RGBA5551              (((psram_oli_100x100_RGBA5551 + g_i32Oli100x100RGBA5551Length + 8) >> 3) << 3)
//#define psram_oli_100x100_ARGB8888              (((psram_oli_390x390_RGBA5551 + oli_390x390_rgba5551_length + 8) >> 3) << 3)
#define psram_oli_100x100_ARGB8888              MSPI_XIP_BASE_ADDRESS
#define psram_oli_100x100_RGBA5650              (((psram_oli_100x100_ARGB8888 + g_i32Oli100x100ARGB8888Length + 8) >> 3) << 3)
#define psram_oli_100x100_TSC6                  (((psram_oli_100x100_RGBA5650 + g_i32Oli100x100RGBA565Length + 8) >> 3) << 3)

#define psram_oli_100x100_RGB24                 (((psram_oli_100x100_TSC6 + g_i32Oli100x100Tsc6Length + 8) >> 3) << 3)
#define oli_100x100_rgb24_length                (100*100*3)

#define psram_oli_360x360_RGB24                 (((psram_oli_100x100_RGB24 + oli_100x100_rgb24_length + 8) >> 3) << 3)
#define oli_360x360_rgb24_length                (360*360*3)

#define psram_oli_360x360_ARGB8888              (((psram_oli_360x360_RGB24 + oli_360x360_rgb24_length + 8) >> 3) << 3)
#define psram_oli_360x360_RGBA5650              (((psram_oli_360x360_ARGB8888 + g_i32Oli360x360ARGB8888Length + 8) >> 3) << 3)
#define psram_oli_360x360_TSC6                  (((psram_oli_360x360_RGBA5650 + g_i32Oli360x360RGBA565Length + 8) >> 3) << 3)

#define psram_oli_20x250_ARGB8888               (((psram_oli_360x360_TSC6 + g_i32Oli360x360Tsc6Length + 8) >> 3) << 3)
#define oli_20x250_argb8888_length              (20*250*4)

#define psram_simhei18pt4b                       (((psram_oli_20x250_ARGB8888 + oli_20x250_argb8888_length + 8) >> 3) << 3)

#define PSRAM_FRAMEBUFFER_LOCATION              (((psram_simhei18pt4b  + g_sSimhei18pt4b .bitmap_size + 8) >> 3) << 3)

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define FB_RESX 390
#define FB_RESY 390

#define TEST_NA 0xFF
#define TEST_ITEM_TOTAL  (sizeof(g_sTestItemList)/sizeof(TestItem_t))

typedef enum
{
    TEST_UNKNOWN = 0,
    TEST_FILL,
    TEST_COPY,
    TEST_BLEND,
    TEST_BLEND_FONT,    //!< blending fond is different from blending texture
    TEST_SCALE,
    TEST_ROTATE,
    TEST_SCALE_ROTATE,
    TEST_SCREENSHOT,
    TEST_TYPE_END,
}
TestType_t;

typedef enum
{
    FB_SSRAM = 0,
    FB_PSRAM,
    FB_EXTENDEDRAM,
    FB_LOCATION_END,
}
FBLocationType_t;

const char* g_pui8TestTypeStr[TEST_TYPE_END] =
{
   "unknown", "fill", "copy", "blend", "blend", "scale", "rotate", "rotate+scale", "screenshot",
};

const char* g_pui8FBLocationStr[FB_LOCATION_END] =
{
   "SSRAM", "PSRAM", "EXTENDED_RAM",
};

typedef struct
{
    TestType_t sTestTyoe;          //!< test type
    FBLocationType_t sFBLocation;  //!< frame buffer location
    nema_tex_format_t sTexFormat;  //!< texture format
    nema_tex_format_t sFBFormat;   //!< framebuffer format
    nemadc_format_t sDCFormat;     //!< dc format
    bool bind_texture;
    bool bind_font;
    uint8_t ui8SamplingMode;        //!< NEMA_FILTER_PS or  NEMA_FILTER_BL
    uint32_t ui32SrcWidth;            //!< texture width
    uint32_t ui32SrcHight;            //!< texture hight
    uint32_t ui32DesWidth;            //!< texture width when it was blit/blend to framebuffer
    uint32_t ui32DesHight;            //!< texture hight when it was blit/blend to framebuffer
    uint32_t ui32Angle;                //!< texture rotation ui32Angle when it was blit/blend to framebuffer
}
TestItem_t;

//*****************************************************************************
//
// Test item list
//
//*****************************************************************************
const TestItem_t g_sTestItemList[] =
{
    //!< sTestTyoe,sFBLocation,sTexFormat,sFBFormat,sDCFormat,bind_texture,bind_font,ui8SamplingMode,ui32SrcWidth,ui32SrcHight,ui32DesWidth,ui32DesHight,ui32Angle,
    {TEST_UNKNOWN,      FB_SSRAM, NEMA_XRGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, false, false, TEST_NA,         100, 100, 100, 100, 0, },
    //!< The first test     was used to warm up the GPU, its result will not be print, leavfalse, e it alone.
    {TEST_FILL,         FB_SSRAM, NEMA_XRGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, false, false, TEST_NA,         100, 100, 100, 100, 0, },
    {TEST_FILL,         FB_SSRAM, NEMA_RGB565,   NEMA_RGB565,   NEMADC_RGB565,   false, false, TEST_NA,         100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_SSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_SSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_SSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_SSRAM, NEMA_RGB24,    NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_SSRAM, NEMA_RGB565,   NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_SSRAM, NEMA_TSC6,     NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_BLEND,        FB_SSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_BLEND,        FB_SSRAM, NEMA_ARGB8888, NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_BLEND_FONT,   FB_SSRAM, NEMA_A4,       NEMA_XRGB8888, NEMADC_ARGB8888, false, true,  TEST_NA,         36,  36,  36,  36,  0, },
    {TEST_BLEND_FONT,   FB_SSRAM, NEMA_A4,       NEMA_RGB565,   NEMADC_RGB565,   false, true,  TEST_NA,         36,  36,  36,  36,  0, },
    {TEST_SCALE,        FB_SSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_SSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_SSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_SSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_SSRAM, NEMA_ARGB8888, NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_SSRAM, NEMA_RGB24,    NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_SSRAM, NEMA_RGB565,   NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_SSRAM, NEMA_TSC6,     NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_ROTATE,       FB_SSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  20,  250, 20,  250, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  20,  250, 20,  250, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_ARGB8888, NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  20,  250, 20,  250, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_ARGB8888, NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  20,  250, 20,  250, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB24,    NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB24,    NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB565,   NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB565,   NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_TSC6,     NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_TSC6,     NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_SSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  20,  250, 360, 360, 1 },
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 1 },
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 1 },
    {TEST_ROTATE,       FB_SSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 1 },
    {TEST_ROTATE,       FB_SSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  20,  250, 360, 360, 44},
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 44},
    {TEST_ROTATE,       FB_SSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 44},
    {TEST_ROTATE,       FB_SSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 44},
    {TEST_SCALE_ROTATE, FB_SSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL, 100, 100, 115, 115, 87},
    /*The following test items put frambuffer at PSRAM*/
    {TEST_FILL,         FB_PSRAM, NEMA_XRGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, false, false, TEST_NA,         100, 100, 100, 100, 0, },
    {TEST_FILL,         FB_PSRAM, NEMA_RGB565,   NEMA_RGB565,   NEMADC_RGB565,   false, false, TEST_NA,         100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_PSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_PSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_PSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_PSRAM, NEMA_RGB24,    NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_PSRAM, NEMA_RGB565,   NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_COPY,         FB_PSRAM, NEMA_TSC6,     NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_BLEND,        FB_PSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_BLEND,        FB_PSRAM, NEMA_ARGB8888, NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  100, 100, 100, 100, 0, },
    {TEST_BLEND_FONT,   FB_PSRAM, NEMA_A4,       NEMA_XRGB8888, NEMADC_ARGB8888, false, true,  TEST_NA,         36,  36,  36,  36,  0, },
    {TEST_BLEND_FONT,   FB_PSRAM, NEMA_A4,       NEMA_RGB565,   NEMADC_RGB565,   false, true,  TEST_NA,         36,  36,  36,  36,  0, },
    {TEST_SCALE,        FB_PSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_PSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_PSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_PSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_PSRAM, NEMA_ARGB8888, NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_PSRAM, NEMA_RGB24,    NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_PSRAM, NEMA_RGB565,   NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_SCALE,        FB_PSRAM, NEMA_TSC6,     NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  100, 100, 115, 115, 0, },
    {TEST_ROTATE,       FB_PSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  20,  250, 20,  250, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  20,  250, 20,  250, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_ARGB8888, NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  20,  250, 20,  250, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_ARGB8888, NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  20,  250, 20,  250, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB24,    NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB24,    NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB565,   NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB565,   NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_TSC6,     NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_TSC6,     NEMA_RGB565,   NEMADC_RGB565,   true,  false, NEMA_FILTER_BL,  360, 360, 360, 360, 87},
    {TEST_ROTATE,       FB_PSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  20,  250, 360, 360, 1 },
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 1 },
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 1 },
    {TEST_ROTATE,       FB_PSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 1 },
    {TEST_ROTATE,       FB_PSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL,  20,  250, 360, 360, 44},
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB24,    NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 44},
    {TEST_ROTATE,       FB_PSRAM, NEMA_RGB565,   NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 44},
    {TEST_ROTATE,       FB_PSRAM, NEMA_TSC6,     NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_PS,  360, 360, 360, 360, 44},
    {TEST_SCALE_ROTATE, FB_PSRAM, NEMA_ARGB8888, NEMA_XRGB8888, NEMADC_ARGB8888, true,  false, NEMA_FILTER_BL, 100, 100, 115, 115, 87},
};

//*****************************************************************************
//
//! @brief Convert color formate to string
//!
//! @return char pointer.
//
//*****************************************************************************
char *
format2string(nema_tex_format_t sFormat)
{
    char* ui8Str;

    switch (sFormat)
    {
        case NEMA_ARGB8888:
            ui8Str = "ARGB8888";
            break;
        case NEMA_XRGB8888:
            ui8Str = "XRGB8888";
            break;
        case NEMA_RGB24:
            ui8Str = "RGB888";
            break;
        case NEMA_RGB565:
            ui8Str = "RGB565";
            break;
        case NEMA_TSC6:
            ui8Str = "TSC6";
            break;
        case NEMA_A4:
            ui8Str = "A4";
            break;
        default:
            ui8Str = "unknown";
            break;
    }

    return ui8Str;
}

//*****************************************************************************
//
//! @brief Map texture by width, hight and color_format
//!
//! @return uintptr_t pointer.
//
//*****************************************************************************
uintptr_t
map_texture(uint32_t ui32Width, uint32_t ui32Hight, nema_tex_format_t sFormat)
{
    uintptr_t texture = (uintptr_t)NULL;

    switch (sFormat)
    {
        case NEMA_ARGB8888:
            if ((ui32Width == 100) && (ui32Hight == 100))
            {
                texture = psram_oli_100x100_ARGB8888;
            }
            else if ((ui32Width == 360) && (ui32Hight == 360))
            {
                texture = psram_oli_360x360_ARGB8888;
            }
            else if ((ui32Width == 20) && (ui32Hight == 250))
            {
                texture = psram_oli_20x250_ARGB8888;
            }
            break;
        case NEMA_RGB24:
            if ((ui32Width == 100) && (ui32Hight == 100))
            {
                texture = psram_oli_100x100_RGB24;
            }
            else if ((ui32Width == 360) && (ui32Hight == 360))
            {
                texture = psram_oli_360x360_RGB24;
            }
            break;
        case NEMA_RGB565:
            if ((ui32Width == 100) && (ui32Hight == 100))
            {
                texture = psram_oli_100x100_RGBA5650 ;
            }
            else if ((ui32Width == 360) && (ui32Hight == 360))
            {
                texture = psram_oli_360x360_RGBA5650;
            }
            break;
        case NEMA_TSC6:
            if ((ui32Width == 100) && (ui32Hight == 100))
            {
                texture = psram_oli_100x100_TSC6;
            }
            else if ((ui32Width == 360) && (ui32Hight == 360))
            {
                texture = psram_oli_360x360_TSC6;
            }
            break;
        default:
            break;
    }

    return texture;
}

//*****************************************************************************
//
//! @brief Map texture by width and format
//!
//! @return nema_font_t pointer.
//
//*****************************************************************************
nema_font_t*
map_font(uint32_t ui32Width, nema_tex_format_t sFormat)
{
    nema_font_t* psFont = NULL;

    switch(sFormat)
    {
        case NEMA_A4:
            if (ui32Width == 36)
            {
                psFont = &g_sSimhei18pt4b ;
            }
            break;
        default:
            break;
    }

    return psFont;
}

//*****************************************************************************
//
//! @brief Set up GP, DC and panel.
//!
//! @return None.
//
//*****************************************************************************
int32_t
test_setup(void)
{
    int32_t i32Ret;
    uint16_t ui16PanelResX = g_sDispCfg[g_eDispType].ui32PanelResX; //!< panel's max resolution
    uint16_t ui16PanelResY = g_sDispCfg[g_eDispType].ui32PanelResY; //!< panel's max resolution
    uint32_t ui32MipiCfg = MIPICFG_8RGB888_OPT0; //!< default config
    uint16_t ui16MinX, ui16MinY;
    //
    // Initialize NemaGFX
    //
    i32Ret = nema_init();
    if (i32Ret != 0)
    {
        am_util_stdio_printf("GPU init failed!\n");
        return i32Ret;
    }
    //
    // Initialize Nema|dc
    //
    i32Ret = nemadc_init();
    if (i32Ret != 0)
    {
        am_util_stdio_printf("DC init failed!\n");
        return i32Ret;
    }

    if ((g_sDispCfg[g_eDispType].eInterface == IF_DSI) || (g_sDispCfg[g_eDispType].bUseDPHYPLL == true))
    {
        uint8_t ui8LanesNum = g_sDsiCfg.ui8NumLanes;
        uint8_t ui8DbiWidth = g_sDsiCfg.eDbiWidth;
        uint32_t ui32FreqTrim = g_sDsiCfg.eDsiFreq;
        pixel_format_t eFormat = FMT_RGB888;
        if (am_hal_dsi_para_config(ui8LanesNum, ui8DbiWidth, ui32FreqTrim) != 0)
        {
            return -3;
        }
        switch (eFormat)
        {
            case FMT_RGB888:
                if (ui8DbiWidth == 16)
                {
                    ui32MipiCfg = MIPICFG_16RGB888_OPT0;
                }
                if (ui8DbiWidth == 8)
                {
                    ui32MipiCfg = MIPICFG_8RGB888_OPT0;
                }
                break;

            case FMT_RGB565:
                if (ui8DbiWidth == 16)
                {
                    ui32MipiCfg = MIPICFG_16RGB565_OPT0;
                }
                if (ui8DbiWidth == 8)
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
    //
    // Set the display region to center
    //
    if (FB_RESX > ui16PanelResX)
    {
        ui16MinX = 0; //!< set the minimum value to 0
    }
    else
    {
        ui16MinX = (ui16PanelResX - FB_RESX) >> 1;
        ui16MinX = (ui16MinX >> 1) << 1;
    }

    if (FB_RESY > ui16PanelResY)
    {
        ui16MinY = 0; //!< set the minimum value to 0
    }
    else
    {
        ui16MinY = (ui16PanelResY - FB_RESY) >> 1;
        ui16MinY = (ui16MinY >> 1) << 1;
    }
    //
    // Initialize the display
    //
    switch (g_sDispCfg[g_eDispType].eInterface)
    {
        case IF_SPI4:
            am_devices_nemadc_rm67162_init(MIPICFG_SPI4, MIPICFG_1RGB565_OPT0, FB_RESX, FB_RESY, ui16MinX, ui16MinY);
            break;
        case IF_DSPI:
            am_devices_nemadc_rm67162_init(MIPICFG_DSPI | MIPICFG_SPI4, MIPICFG_2RGB888_OPT0, FB_RESX, FB_RESY, ui16MinX, ui16MinY);
            break;
        case IF_QSPI:
            am_devices_nemadc_rm67162_init(MIPICFG_QSPI | MIPICFG_SPI4, MIPICFG_4RGB565_OPT0, FB_RESX, FB_RESY, ui16MinX, ui16MinY);
            break;
        case IF_DSI:
            am_devices_dsi_rm67162_init(ui32MipiCfg, FB_RESX, FB_RESY, ui16MinX, ui16MinY);
            break;
        default:
            ; //NOP
    }
    return 0;
}

//*****************************************************************************
//
//! @brief Load textures from MRAM to PSRAM.n
//!
//! @return None.
//
//*****************************************************************************
void
load_texture(void)
{
    //!< load textures from MRAM to PSRAM
//    memcpy((void*)psram_oli_100x100_RGBA5551,   g_ui8Oli100x100RGBA5551,   g_i32Oli100x100RGBA5551Length);
//    memcpy((void*)psram_oli_390x390_RGBA5551 ,  oli_390x390_rgba5551,   oli_390x390_rgba5551_length);
    memcpy((void*)psram_oli_100x100_ARGB8888,   g_ui8Oli100x100ARGB8888,   g_i32Oli100x100ARGB8888Length);
    memcpy((void*)psram_oli_100x100_RGBA5650,   g_ui8Oli100x100RGBA565,    g_i32Oli100x100RGBA565Length);
    memcpy((void*)psram_oli_100x100_TSC6,       g_ui8Oli100x100Tsc6,       g_i32Oli100x100Tsc6Length);
    memcpy((void*)psram_oli_360x360_ARGB8888,   g_ui8Oli360x360ARGB8888,   g_i32Oli360x360ARGB8888Length);
    memcpy((void*)psram_oli_360x360_RGBA5650,   g_ui8Oli360x360RGBA565,    g_i32Oli360x360RGBA565Length);
    memcpy((void*)psram_oli_360x360_TSC6,       g_ui8Oli360x360Tsc6,       g_i32Oli360x360Tsc6Length);

    //!< load font bitmap from MRAM to PSRAM
    memcpy((void*)psram_simhei18pt4b ,           g_sSimhei18pt4b .bitmap,    g_sSimhei18pt4b .bitmap_size);
    g_sSimhei18pt4b .bo.base_phys = psram_simhei18pt4b ;
    g_sSimhei18pt4b .bo.base_virt = (void*)g_sSimhei18pt4b .bo.base_phys;

    //!< We don't have a RGB24 texture, so we just create one witch was filled by 0xFF,
    //!< you will see all white when it was displayed on the panel
    memset((void*)psram_oli_100x100_RGB24, 0xFF, oli_100x100_rgb24_length);
    memset((void*)psram_oli_360x360_RGB24, 0xFF, oli_360x360_rgb24_length);
    memset((void*)psram_oli_20x250_ARGB8888, 0xFF, oli_20x250_argb8888_length);

}


//*****************************************************************************
//
//! @brief Init framebuffer.
//!
//! @return int32_t.
//
//*****************************************************************************
int32_t
init_framebuffer(img_obj_t* psFB,
                 uint32_t ui32Width,
                 uint32_t ui32Hight,
                 nema_tex_format_t sFormat,
                 FBLocationType_t sFBLocation)
{
    psFB->w = ui32Width;
    psFB->h = ui32Hight;
    psFB->format = sFormat;
    psFB->stride = nema_stride_size(sFormat, 0, ui32Width);

    //
    // Alloc frame buffer space
    //
    switch (sFBLocation)
    {
        case FB_SSRAM:
            psFB->bo = nema_buffer_create(psFB->stride * psFB->h);
            (void)nema_buffer_map(&psFB->bo);
            break;
        case FB_PSRAM:
            //!< TODO:Implement a memory malloc algorithm for PSRAM
            psFB->bo.base_phys = PSRAM_FRAMEBUFFER_LOCATION;
            psFB->bo.base_virt = (void*)psFB->bo.base_phys;
            break;
        case FB_EXTENDEDRAM:
            am_util_stdio_printf("Can't create frame buffer at extended ram, to be implement!\n");
            psFB->bo.base_phys = (uintptr_t)NULL;
            psFB->bo.base_virt = (void*)psFB->bo.base_phys;
            break;
        default:
            am_util_stdio_printf("Unkown frame buffer location!\n");
            psFB->bo.base_phys = (uintptr_t)NULL;
            psFB->bo.base_virt = (void*)psFB->bo.base_phys;
            break;

    }

    if ((void*)psFB->bo.base_phys == NULL)
    {
        am_util_stdio_printf("frame buffer malloc failed!\n");
        return -1;
    }

    return 0;
}

//*****************************************************************************
//
//! @brief Init layer, related parameter will copy from fb.
//!
//! @return None.
//
//*****************************************************************************
void
init_layer_from_framebuffer(nemadc_layer_t* psLayer, img_obj_t* psFB, nemadc_format_t sFormat)
{
    psLayer->startx        = 0;
    psLayer->sizex         = psFB->w;
    psLayer->resx          = psFB->w;
    psLayer->starty        = 0;
    psLayer->sizey         = psFB->h;
    psLayer->resy          = psFB->h;
    psLayer->stride        = psFB->stride;
    psLayer->format        = sFormat;
    psLayer->blendmode     = NEMADC_BL_SRC;
    psLayer->baseaddr_phys = psFB->bo.base_phys;
    psLayer->baseaddr_virt = psFB->bo.base_virt;
    psLayer->buscfg        = 0;
    psLayer->alpha         = 0xff;
}


//*****************************************************************************
//
//! @brief refresh screen.
//!
//! @return None.
//
//*****************************************************************************
void
flush_screen(void)
{
    if (g_sDispCfg[g_eDispType].eInterface == IF_DSI)
    {
        dsi_send_frame_single(NEMADC_OUTP_OFF);
    }
    else
    {
        nemadc_send_frame_single();
    }
}

//!< rotate
void
nema_blit_rotate_pivot(float w, float h, float cx, float cy, float px, float py, float degrees_cw)
{
    float x0 = -px;
    float y0 = -py;
    float x1 = x0 + w;
    float y1 = y0;
    float x2 = x0 + w;
    float y2 = y0 + h;
    float x3 = x0;
    float y3 = y0 + h;

    nema_matrix3x3_t m;
    nema_mat3x3_load_identity(m);
    nema_mat3x3_rotate(m, degrees_cw);
    nema_mat3x3_translate(m, cx, cy);
    nema_mat3x3_mul_vec(m, &x0, &y0);
    nema_mat3x3_mul_vec(m, &x1, &y1);
    nema_mat3x3_mul_vec(m, &x2, &y2);
    nema_mat3x3_mul_vec(m, &x3, &y3);
    nema_blit_quad_fit(x0, y0,
                       x1, y1,
                       x2, y2,
                       x3, y3);
}

int32_t
nemagfx_performance_test(void)
{
    int32_t i;
    int32_t i32Ret;
    static img_obj_t sFB;
    static nemadc_layer_t sLayer;
    static nema_cmdlist_t sCL;
    const TestItem_t* psTestItem;
    float time_start;
    float time_end;
    float time_used;
    float speed_pps;
    uint32_t ui32StartX;
    uint32_t ui32StartY;
    uintptr_t texture;
    nema_font_t* psFont;
    char ui8Str[] = "鱀\n";
    //
    // Set up GPU, DC
    //
    i32Ret = test_setup();
    if ( i32Ret != 0 )
    {
        return i32Ret;
    }
    //
    // Set up texture
    //
    load_texture();
    //
    // Create CL
    //
    sCL = nema_cl_create_sized(0x400);
    //
    // Set DC timing
    //
    nemadc_timing(FB_RESX, 4, 10, 10, FB_RESY, 10, 50, 10);

    am_util_stdio_printf("type,FB_Location,Tex_Format,FB_Format,SamplingMode,Src_Width,Src_Hight,Des_Width,Des_Hight,Angle,time,speed_MP/s\n");
    //
    // run the test
    //
    for (i = 0; i < TEST_ITEM_TOTAL; i++)
    {
        psTestItem = &g_sTestItemList[i];
        //
        // Init framebuffer and layer
        //
        init_framebuffer(&sFB, FB_RESX, FB_RESY, psTestItem->sFBFormat, psTestItem->sFBLocation);
        init_layer_from_framebuffer(&sLayer, &sFB, psTestItem->sDCFormat);
        //
        // Set DC layer
        //
        nemadc_set_layer(0, &sLayer);
        //
        // rewind and bind the CL
        //
        nema_cl_rewind(&sCL);
        nema_cl_bind(&sCL);
        //
        // bind framebuffer as destination texture
        //
        nema_bind_dst_tex(sFB.bo.base_phys, FB_RESX, FB_RESY, sFB.format, sFB.stride);
        //
        // set clip
        //
        nema_set_clip(0, 0, FB_RESX, FB_RESY);
        //
        // clear the frambuffer
        //
        nema_clear(0);
        //
        // submit the CL
        //
        nema_cl_submit(&sCL);
        //
        // wait for GPU
        //
        nema_cl_wait(&sCL);
        //
        // rewind and bind the CL
        //
        nema_cl_rewind(&sCL);
        nema_cl_bind(&sCL);
        //
        // bind framebuffer as destination texture
        //
        nema_bind_dst_tex(sFB.bo.base_phys, FB_RESX, FB_RESY, sFB.format, sFB.stride);
        //
        // set clip
        //
        nema_set_clip(0, 0, FB_RESX, FB_RESY);
        //
        // bind source texture
        //
        if (psTestItem->bind_texture == true)
        {
            texture = map_texture(psTestItem->ui32SrcWidth, psTestItem->ui32SrcHight, psTestItem->sTexFormat);

            if (texture == (uintptr_t)NULL)
            {
               am_util_stdio_printf("Cann't find source texture!\n");
               return -1;
            }

            nema_bind_src_tex(texture,
                              psTestItem->ui32SrcWidth,
                              psTestItem->ui32SrcHight,
                              psTestItem->sTexFormat,
                              -1,
                              psTestItem->ui8SamplingMode);
        }
        //
        // bind font
        //
        if (psTestItem->bind_font == true)
        {
            psFont = map_font(psTestItem->ui32SrcWidth, psTestItem->sTexFormat);

            if (psFont == NULL)
            {
               am_util_stdio_printf("Cann't find psFont!\n");
               return -1;
            }

            nema_bind_font(psFont);
        }

        //!< start point
        ui32StartX = (FB_RESX-psTestItem->ui32SrcWidth) / 2;
        ui32StartY = (FB_RESX-psTestItem->ui32SrcHight) / 2;
        //
        // create the cl
        //
        switch(psTestItem->sTestTyoe)
        {
            case TEST_TYPE_END:
                break;

            case TEST_UNKNOWN:
            case TEST_FILL:
                nema_set_blend_fill(NEMA_BL_SRC);
                nema_fill_rect(ui32StartX, ui32StartY, psTestItem->ui32SrcWidth, psTestItem->ui32SrcHight, 0xAABBCC00);
                break;

            case TEST_COPY:
                nema_set_blend_blit(NEMA_BL_SRC);
                nema_blit(ui32StartX, ui32StartY);
                break;

            case TEST_BLEND:
                nema_set_blend_blit(NEMA_BL_SRC);
                nema_blit(ui32StartX, ui32StartY);
                break;

            case TEST_BLEND_FONT:
                nema_print(ui8Str, ui32StartX, ui32StartY, psTestItem->ui32DesWidth, psTestItem->ui32DesHight, 0xff00ffffU, NEMA_ALIGNX_LEFT   | NEMA_TEXT_WRAP | NEMA_ALIGNY_TOP);
                break;

            case TEST_SCALE:
                nema_set_blend_blit(NEMA_BL_SRC);
                nema_blit_rect_fit(ui32StartX, ui32StartY, psTestItem->ui32DesWidth, psTestItem->ui32DesHight);
                break;

            case TEST_ROTATE:
                nema_set_blend_blit(NEMA_BL_SRC);
                nema_blit_rotate_pivot(psTestItem->ui32SrcWidth,
                                   psTestItem->ui32SrcHight,
                                   FB_RESX / 2,
                                   FB_RESY / 2,
                                   psTestItem->ui32SrcWidth / 2,
                                   psTestItem->ui32SrcHight / 2,
                                   psTestItem->ui32Angle);
                break;

            case TEST_SCALE_ROTATE:
                nema_set_blend_blit(NEMA_BL_SRC);
                nema_blit_rotate_pivot(psTestItem->ui32DesWidth,
                                   psTestItem->ui32DesHight,
                                   FB_RESX / 2,
                                   FB_RESY / 2,
                                   psTestItem->ui32DesWidth / 2,
                                   psTestItem->ui32DesHight / 2,
                                   psTestItem->ui32Angle);
                break;

            case TEST_SCREENSHOT:
                break;
        }
        //
        // submit the CL
        //
        nema_cl_submit(&sCL);
        //
        // wait for GPU
        //
        time_start = nema_get_time();
        nema_cl_wait(&sCL);
        time_end = nema_get_time();
        //
        // flush screen
        //
        flush_screen();
        //
        // conculate result and output
        //
        time_used = (time_end - time_start) * 1000.0;
        speed_pps = (psTestItem->ui32DesWidth * psTestItem->ui32DesHight) / time_used / 1000.0;

        if (psTestItem->sTestTyoe != TEST_UNKNOWN)
        {
          am_util_stdio_printf("%s,%s,%s,%s,%s,%d,%d,%d,%d,%d,%.3f,%.3f\n",
                              g_pui8TestTypeStr[psTestItem->sTestTyoe],
                              g_pui8FBLocationStr[psTestItem->sFBLocation],
                              format2string(psTestItem->sTexFormat),
                              format2string(psTestItem->sFBFormat),
                              (psTestItem->ui8SamplingMode == NEMA_FILTER_PS ? "Point" : "Bilinear"),
                              psTestItem->ui32SrcWidth,
                              psTestItem->ui32SrcHight,
                              psTestItem->ui32DesWidth,
                              psTestItem->ui32DesHight,
                              psTestItem->ui32Angle,
                              time_used,
                              speed_pps);
        }

        //
        // Release framebuffer
        //
        nema_buffer_unmap(&sFB.bo);
        nema_buffer_destroy(&sFB.bo);
    }
    //
    // calculate the result
    //
    return 0;
}

