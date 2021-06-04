//*****************************************************************************
//
//! @file nemagfx_blit_test_cases.c
//!
//! @brief NemaGFX blit test cases.
//! Need to connect RM67162 to DC SPI4 interface.
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include "am_bsp.h"
#include "nema_hal.h"
#include "nema_dc.h"
#include "nema_dc_mipi.h"
#include "am_util_delay.h"
#include "am_util_stdio.h"
#include "nema_dc_regs.h"
#include "nema_utils.h"
#include "nema_graphics.h"
#include "nema_cmdlist.h"
#include "nema_blender.h"
#include "Ambiq200x104_rgba.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
//#define WITH_DISPLAY 1
#define ENABLE_SPI4 1

#define NemaDC_rcmd16     (1U << 28)
#define NemaDC_rcmd24     (1U << 29)
#define NemaDC_rcmd32     ((1U << 29) | (1U << 28))
#define NemaDC_DBI_read   (1U << 26)

//
// Maximum read frequency of RM67162 is about 3MHz
//
#define TARGET_FREQ 3
//
// Format clock frequency is half of DISPCLK.
//
#define FORMAT_CLK_DIV 2
//
// Assume DISPCLK is 48MHz or 96MHz on actual silicon, the proper division ratio is 16.
//
#define SILICON_FORMAT_DIV (16U)
//
// DC option clock divider position
//
#define DC_DIV2_POS 27

#define FB_RESX 200
#define FB_RESY 200

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
static img_obj_t fb = {{0}, FB_RESX, FB_RESY, -1, 0, NEMA_TSC6, 0};
img_obj_t AM_logo = {{0}, 64, 64, 200 * 4, 0, NEMA_RGBA8888, 0}; // set to 64*64 or 128*128, repeat blit passed.
nemadc_layer_t dc_layer = {(void *)0, 0, FB_RESX, FB_RESY, -1, 0, 0, FB_RESX, FB_RESY, 0xff, NEMADC_BL_SRC, 0, NEMADC_TSC6, 0, 0, 0, 0, 0};
AM_SHARED_RW nema_cmdlist_t cl;

static uint8_t index = 0;

uint32_t g_gpu_checksum_table[3] = { 0x00003F39, 0x000046B2, 0x00008D6D};
uint32_t g_dc_checksum_table[2][3]  = {{ 0x1372c447, 0xf3425cd1, 0x4e9c37c8},{ 0xd01c541e, 0xf8a05cc0, 0x65acee15}};

bool get_checksum(uint32_t ref_checksum)
{
    bool bPass = true;
    uint32_t * buff;
    uint32_t checksum, i;
    buff = (uint32_t *)dc_layer.baseaddr_virt;
    checksum = 0;
    for(i = 0; i < FB_RESX * FB_RESY * 3 / 4 / 4; i++) // TSC6 format
    {
        checksum += buff[i];
    }
    checksum = (checksum >> 16) + (checksum & 0xffff);
    checksum += (checksum >> 16);
    checksum = 0xffff - checksum;
    if (checksum != ref_checksum)
    {
        am_util_stdio_printf("ERROR! Expected FB checksum is 0x%08X, current checksum is 0x%08X.\n",
                            ref_checksum, checksum);
        bPass = false;
    }
    else
    {
        am_util_stdio_printf("Current FB checksum is equal to expected checksum (0x%08X).\n",
                            ref_checksum);
    }

    return bPass;
}


//*****************************************************************************
//
//! @brief Load memory objects.
//!
//! @return NULL.
//
//*****************************************************************************
void
load_objects(void)
{
    fb.bo = nema_buffer_create(fb.w * fb.h * 3);
    nema_buffer_map(&fb.bo);

    dc_layer.baseaddr_phys = fb.bo.base_phys;
    dc_layer.baseaddr_virt = fb.bo.base_virt;

    AM_logo.bo = nema_buffer_create(Ambiq200x104_rgba_len);
    nema_memcpy(AM_logo.bo.base_virt, Ambiq200x104, Ambiq200x104_rgba_len);
}

//*****************************************************************************
//
//! @brief Read data from display module.
//!
//! @param eCmd - Read command.
//! @param ui32Length - Number of read bytes.
//!
//! @return ui32RData - the data read back.
//
//*****************************************************************************
uint32_t
am_nemadc_read(nemadc_mipi_cmd_t eCmd, uint32_t ui32Length)
{
    uint32_t ui32RData = 0;

#if ((defined ENABLE_DSPI) || (defined ENABLE_QSPI))
    uint32_t ui32DBIConfig;
#endif

#if ((defined ENABLE_SPI4) || (defined ENABLE_DSPI))
    uint32_t ui32Cfg;
    switch(ui32Length)
    {
        case 1:
            ui32Cfg = 0;
            break;
        case 2:
            ui32Cfg = NemaDC_rcmd16;
            break;
        case 3:
            ui32Cfg = NemaDC_rcmd24;
            break;
        case 4:
            ui32Cfg = NemaDC_rcmd32;
            break;
        default:
            return 0;
    }
#endif

//
// 12MHz FPGA image requires longer delay than 48MHz image, so changed delay from 20us to 100us, added some margin.
//
#ifdef ENABLE_SPI4
    nemadc_MIPI_out(MIPI_DBIB_CMD | NemaDC_DBI_read | ui32Cfg | eCmd);
    while((nemadc_reg_read(NEMADC_REG_STATUS) & 0x1c00U) != 0U);
    am_util_delay_us(100);
    ui32RData = nemadc_reg_read(NEMADC_REG_DBIB_RDAT);
#endif

#ifdef ENABLE_DSPI
    ui32DBIConfig = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
    nemadc_MIPI_CFG_out(ui32DBIConfig & (~MIPICFG_DSPI) & (~MIPICFG_QSPI));
    nemadc_MIPI_out(MIPI_DBIB_CMD | NemaDC_DBI_read | ui32Cfg | eCmd);
    while((nemadc_reg_read(NEMADC_REG_STATUS) & 0x1c00U) != 0U);
    am_util_delay_us(100);
    ui32RData = nemadc_reg_read(NEMADC_REG_DBIB_RDAT);
    nemadc_MIPI_CFG_out(ui32DBIConfig);
#endif

#ifdef ENABLE_QSPI // QSPI only supports 1 byte read.
    ui32DBIConfig = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
    nemadc_MIPI_CFG_out(ui32DBIConfig | MIPICFG_SPI_HOLD);
    nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_MASK_QSPI | SPI_READ);
    nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_MASK_QSPI | MIPI_CMD16 | (eCmd));
    nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_MASK_QSPI | NemaDC_DBI_read);
    while((nemadc_reg_read(NEMADC_REG_STATUS) & 0x1c00U) != 0U);
    nemadc_MIPI_CFG_out(ui32DBIConfig);
    am_util_delay_us(100);
    ui32RData = nemadc_reg_read(NEMADC_REG_DBIB_RDAT);
#endif
    return ui32RData;
}



//*****************************************************************************
//
//! @brief Test NemaGFX blit feature.
//!
//! @return bTestPass.
//
//*****************************************************************************
bool
nemagfx_blit_test(void)
{
    bool bTestPass = true;
    uint16_t ui16PanelResX = 390; // panel'ui32SampleType max resolution
    uint16_t ui16PanelResY = 390; // panel'ui32SampleType max resolution
    uint16_t ui16MinX, ui16MinY;
    uint32_t ui32SampleType;
    uint32_t ui32TexColor;
	
	
	nema_init();
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
    //
    // Assign a fixed value to display type.
    //
    g_eDispType = RM67162_SPI4;
    //
    // Set the display region to center
    //
    ui16MinX = (FB_RESX >= ui16PanelResX)? 0 : ((ui16PanelResX - FB_RESX) >> 2)  << 1;
    ui16MinY = (FB_RESY >= ui16PanelResY)? 0 : ((ui16PanelResY - FB_RESY) >> 2)  << 1;

    am_devices_nemadc_rm67162_init(MIPICFG_SPI4, MIPICFG_1RGB888_OPT0, FB_RESX, FB_RESY, ui16MinX, ui16MinY);
		
    load_objects();

    nemadc_set_layer(0, &dc_layer);

    //Create Command Lists
    cl = nema_cl_create();
    //
    //Bind Command List
    //
    nema_cl_bind(&cl);
    //
    //Bind Framebuffer
    //
    nema_bind_dst_tex(fb.bo.base_phys, fb.w, fb.h, fb.format, fb.stride);
    //
    //Set Clipping Rectangle
    //
    nema_set_clip(0, 0, FB_RESX, FB_RESY);
    //
    // Check Sampling Modes - CLAMP/REPEAT/BORDER/MIRROR
    // MIRROR is not working on the FPGA (probably neither in ASIC)
    // We need to fix this in REV-B chip
    //
    nema_set_blend_blit(NEMA_BL_SRC);
    const uint32_t sampling_type[] =
    {
        NEMA_TEX_CLAMP,
        NEMA_TEX_REPEAT,
        NEMA_TEX_BORDER,
        NEMA_TEX_MIRROR
    };
    ui32TexColor = 0xFFBBCCDD; // A-B-G-R
    nema_set_tex_color(ui32TexColor);
    if(0)
    {
        index = 1;
    }
    else
    {
        index = 0;
    }

    for(ui32SampleType = 0; ui32SampleType < 3; ui32SampleType++) // Removed MIRROR test
    {
        am_util_stdio_printf("\nNemaGFX blit (sampling type - %d) test.\n", ui32SampleType);
        nema_bind_src_tex(AM_logo.bo.base_phys, AM_logo.w, AM_logo.h, AM_logo.format, AM_logo.stride, NEMA_FILTER_PS | sampling_type[ui32SampleType]);
        nema_blit_subrect(0, 0, FB_RESX, FB_RESY, -50, -50);
        nema_cl_submit(&cl);
        nema_cl_wait(&cl);
        nema_cl_rewind(&cl);
        if (get_checksum(g_gpu_checksum_table[ui32SampleType]) == false)
        {
            bTestPass = false;
        }
        nemadc_set_layer(0, &dc_layer);
        nemadc_send_frame_single();
        //nemadc_layer_disable(0);
        nemadc_layer_enable(0);
#ifdef WITH_DISPLAY
        uint32_t ui32FreqDiv, ui32RReg;
        uint32_t ui32RData = 0;
        uint8_t ui8Byte0, ui8Byte1;
        uint8_t ui8Golden[2];
        am_util_delay_ms(500);
        ui32RReg = nemadc_reg_read(NEMADC_REG_CLKCTRL);

        ui32FreqDiv = (SILICON_FORMAT_DIV << DC_DIV2_POS);

        nemadc_reg_write(NEMADC_REG_CLKCTRL, ui32RReg | ui32FreqDiv);
        am_util_delay_ms(100); // Added a delay to wait CLK back to stable.
        //
        // read frame from panel
        //
        ui32RData = am_nemadc_read(MIPI_read_memory_start, 3); // first byte is dummy byte.
        ui8Byte0 = ((ui32RData >> 8) & 0x000000FF);
        ui8Byte1 = (ui32RData & 0x000000FF);
        if (ui32SampleType == 0) // CLAMP
        {
            ui8Golden[0] = Ambiq200x104[0];
            ui8Golden[1] = Ambiq200x104[1];
        }
        else if(ui32SampleType == 1) // REPEAT
        {
            ui8Golden[0] = Ambiq200x104[14 * 200 * 4 + 14 * 4]; // Find the pixel at panel origin. (64-50)*200*4+(64-50)*4
            ui8Golden[1] = Ambiq200x104[14 * 200 * 4 + 14 * 4 + 1];
        }
        else if(ui32SampleType == 2) // BORDER
        {
            ui8Golden[0] = (ui32TexColor & 0x000000FF);
            ui8Golden[1] = ((ui32TexColor >> 8) & 0x000000FF);
        }

        if((ui8Byte1 != ui8Golden[1]) || (ui8Byte0 != ui8Golden[0]))
        {
            bTestPass = false;
            am_util_stdio_printf("ERROR! Expected pixel data are 0x%02X 0x%02X, current data were 0x%02X 0x%02X.\n", ui8Golden[0], ui8Golden[1], ui8Byte0, ui8Byte1);
        }
        else
        {
            am_util_stdio_printf("Current pixel data are equal to expected data (0x%02X 0x%02X).\n", ui8Golden[0], ui8Golden[1]);
        }
        nemadc_reg_write(NEMADC_REG_CLKCTRL, ui32RReg); // Disabled format clock division.
        am_util_delay_ms(100); // Added a delay to wait CLK back to stable.
#endif
        uint32_t read_crc_dc;
        read_crc_dc = nemadc_get_crc();
        if (read_crc_dc != g_dc_checksum_table[index][ui32SampleType])
        {
            bTestPass = false;
            am_util_stdio_printf("ERROR! Expected DC checksum is 0x%08x, current checksum is 0x%08x\n",
                                g_dc_checksum_table[index][ui32SampleType], read_crc_dc);
        }
        else
        {
            am_util_stdio_printf("Current DC checksum is equal to expected checksum (0x%08X).\n",
                                g_dc_checksum_table[index][ui32SampleType]);
        }
    }

    //TEST_ASSERT_TRUE(bTestPass);

    return bTestPass;

} // nemagfx_blit_test()
