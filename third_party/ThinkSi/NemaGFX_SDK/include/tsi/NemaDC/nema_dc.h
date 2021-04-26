/*******************************************************************************
 * Copyright (c) 2019 Think Silicon S.A.
 *
   Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this header file and/or associated documentation files to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Materials, and to permit persons to whom the Materials are furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Materials.
 *
 * MODIFICATIONS TO THIS FILE MAY MEAN IT NO LONGER ACCURATELY REFLECTS
 * NEMAGFX API. THE UNMODIFIED, NORMATIVE VERSIONS OF THINK-SILICON NEMAGFX
 * SPECIFICATIONS AND HEADER INFORMATION ARE LOCATED AT:
 *   https://think-silicon.com/products/software/nemagfx-api
 *
 *  The software is provided 'as is', without warranty of any kind, express or
 *  implied, including but not limited to the warranties of merchantability,
 *  fitness for a particular purpose and noninfringement. In no event shall
 *  Think Silicon S.A. be liable for any claim, damages or other liability, whether
 *  in an action of contract, tort or otherwise, arising from, out of or in
 *  connection with the software or the use or other dealings in the software.
 ******************************************************************************/


#ifndef NEMA_DC_H__
#define NEMA_DC_H__

#include "nema_sys_defs.h"
#include "nema_dc_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ThinkLCD CNFG register bits
#define NEMADC_CFG_LAYER_EXISTS(i)   (1U << (8 + (i)*4))
#define NEMADC_CFG_LAYER_BLENDER(i)  (1U << (8 + (i)*4 + 1))
#define NEMADC_CFG_LAYER_SCALER(i)   (1U << (8 + (i)*4 + 2))
#define NEMADC_CFG_LAYER_GAMMA(i)    (1U << (8 + (i)*4 + 3))

// Layer Control
//--------------------------------------------------------------------------
#define    NEMADC_LAYER_ENABLE   (1U << 31)   /**< Enable Layer */
typedef enum {
    NEMADC_LAYER_DISABLE     = 0,         /**< Disable Layer */
    NEMADC_FORCE_A           = 1U << 30,   /**< Force Alpha */
    NEMADC_SCALE_NN          = 1U << 29,   /**< Activate Bilinear Filter */
    NEMADC_MODULATE_A        = 1U << 28,   /**< Modulate Alpha */
    NEMADC_LAYER_AHBLOCK     = 1U << 27,   /**< Activate HLOCK signal on AHB DMAs */
    NEMADC_LAYER_GAMMALUT_EN = 1U << 26  /**< Enable Gamma Look Up Table */
} nemadc_layer_ctrl_t;
//--------------------------------------------------------------------------
typedef enum {
    NEMADC_BF_ZERO           = 0x0, /**< Black */
    NEMADC_BF_ONE            = 0x1, /**< White */
    NEMADC_BF_SRCALPHA       = 0x2, /**< Alpha Source */
    NEMADC_BF_GLBALPHA       = 0x3, /**< Alpha Global */
    NEMADC_BF_SRCGBLALPHA    = 0x4, /**< Alpha Source And Alpha Global */
    NEMADC_BF_INVSRCALPHA    = 0x5, /**< Inverted Source */
    NEMADC_BF_INVGBLALPHA    = 0x6, /**< Inverted Global */
    NEMADC_BF_INVSRCGBLALPHA = 0x7, /**< Inverted Source And Global */
    NEMADC_BF_DSTALPHA       = 0xa, /**< Alpha Destination */
    NEMADC_BF_INVDSTALPHA    = 0xb  /**< Inverted Destination */
} nemadc_blend_factors_t;

typedef enum {
    /*                  source factor         destination factor */
    NEMADC_BL_SIMPLE     = (NEMADC_BF_SRCALPHA     | (NEMADC_BF_INVSRCALPHA  <<4)),   /**< Sa * Sa + Da * (1 - Sa) */
    NEMADC_BL_CLEAR      = (NEMADC_BF_ZERO         | (NEMADC_BF_ZERO         <<4)),   /**< 0 */
    NEMADC_BL_SRC        = (NEMADC_BF_ONE          | (NEMADC_BF_ZERO         <<4)),   /**< Sa */
    NEMADC_BL_SRC_OVER   = (NEMADC_BF_ONE          | (NEMADC_BF_INVSRCALPHA  <<4)),   /**< Sa + Da * (1 - Sa) */
    NEMADC_BL_DST_OVER   = (NEMADC_BF_INVDSTALPHA  | (NEMADC_BF_ONE          <<4)),   /**< Sa * (1 - Da) + Da */
    NEMADC_BL_SRC_IN     = (NEMADC_BF_DSTALPHA     | (NEMADC_BF_ZERO         <<4)),   /**< Sa * Da */
    NEMADC_BL_DST_IN     = (NEMADC_BF_ZERO         | (NEMADC_BF_SRCALPHA     <<4)),   /**< Da * Sa */
    NEMADC_BL_SRC_OUT    = (NEMADC_BF_INVDSTALPHA  | (NEMADC_BF_ZERO         <<4)),   /**< Sa * (1 - Da) */
    NEMADC_BL_DST_OUT    = (NEMADC_BF_ZERO         | (NEMADC_BF_INVSRCALPHA  <<4)),   /**< Da * (1 - Sa) */
    NEMADC_BL_SRC_ATOP   = (NEMADC_BF_DSTALPHA     | (NEMADC_BF_INVSRCALPHA  <<4)),   /**< Sa * Da + Da * (1 - Sa) */
    NEMADC_BL_DST_ATOP   = (NEMADC_BF_INVDSTALPHA  | (NEMADC_BF_SRCALPHA     <<4)),   /**< Sa * (1 - Da) + Da * Sa */
    NEMADC_BL_ADD        = (NEMADC_BF_ONE          | (NEMADC_BF_ONE          <<4)),   /**< Sa + Da */
    NEMADC_BL_XOR        = (NEMADC_BF_INVDSTALPHA  | (NEMADC_BF_INVSRCALPHA  <<4))    /**< Sa * (1 - Da) + Da * (1 - Sa) */
} nemadc_blend_mode_t;

//--------------------------------------------------------------------------
enum {
    NEMADC_RGBA5551  = 0x01,  /**< RGBA5551 */
    NEMADC_ABGR8888  = 0x02,  /**< ABGR8888 */
    NEMADC_RGB332    = 0x04,  /**< RGB332 */
    NEMADC_RGB565    = 0x05,  /**< RGB565 */
    NEMADC_BGRA8888  = 0x06,  /**< BGRA8888 */
    NEMADC_L8        = 0x07,  /**< L8 */
    NEMADC_L1        = 0x08,  /**< L1 */
    NEMADC_L4        = 0x09,  /**< L4 */
    NEMADC_YUYV      = 0x0a,  /**< YUYV */
    NEMADC_RGB24     = 0x0b,  /**< RGB24 */
    NEMADC_YUY2      = 0x0c,  /**< YUY2 */
    NEMADC_RGBA8888  = 0x0d,  /**< RGBA8888 */
    NEMADC_ARGB8888  = 0x0e,  /**< ARGB8888 */
    NEMADC_V_YUV420  = 0x10,  /**< V_YUV420 */
    NEMADC_TLYUV420  = 0x11,  /**< TLYUV420 */
    NEMADC_TSC4      = 0x12,  /**< TSC4 */
    NEMADC_TSC6      = 0x13,  /**< TSC6 */
    NEMADC_TSC6A     = 0x14   /**< TSC6A */
};

typedef uint32_t nemadc_format_t;

// Hardware Colour Formats Display Modes
//--------------------------------------------------------------------------
#define  NEMADC_ENABLE (1U << 31)  /**< ENABLE */

typedef enum {
    NEMADC_DISABLE    =  0,        /**< DISABLE */
    NEMADC_CURSOR     =  1U << 30,  /**< CURSOR */
    NEMADC_NEG_V      =  1U << 28,  /**< NEG_V */
    NEMADC_NEG_H      =  1U << 27,  /**< NEG_H */
    NEMADC_NEG_DE     =  1U << 26,  /**< NEG_DE */
    NEMADC_DITHER     =  1U << 24,  /**< DITHER 18-bit */
    NEMADC_DITHER16   =  2U << 24,  /**< DITHER 16-bit */
    NEMADC_DITHER15   =  3U << 24,  /**< DITHER 15-bit */
    NEMADC_SINGLEV    =  1U << 23,  /**< SINGLEV */
    NEMADC_INVPIXCLK  =  1U << 22,  /**< INVPIXCLK */
    NEMADC_PALETTE    =  1U << 20,  /**< PALETTE */
    NEMADC_GAMMA      =  1U << 20,  /**< GAMMA */
    NEMADC_BLANK      =  1U << 19,  /**< BLANK */
    NEMADC_INTERLACE  =  1U << 18,  /**< INTERLACE */
    NEMADC_ONE_FRAME  =  1U << 17,  /**< ONE_FRAME */
    NEMADC_P_RGB3_18B =  1U << 12,  /**< P_RGB3 */
    NEMADC_P_RGB3_18B1=  2U << 12,  /**< P_RGB3 */
    NEMADC_P_RGB3_16B =  3U << 12,  /**< P_RGB3 */
    NEMADC_P_RGB3_16B1=  4U << 12,  /**< P_RGB3 */
    NEMADC_P_RGB3_16B2=  5U << 12,  /**< P_RGB3 */
    NEMADC_CLKOUTDIV  =  1U << 11,  /**< CLKOUTDIV */
    NEMADC_LVDSPADS   =  1U << 10,  /**< LVDSPADS */
    NEMADC_YUVOUT     =  1U << 9,  /**< YUVOUT */
    NEMADC_MIPI_OFF   =  1U << 4,  /**< MIPI_OFF */
    NEMADC_OUTP_OFF   =  1U << 3,  /**< OUTP_OFF */
    NEMADC_LVDS_OFF   =  1U << 2,  /**< LVDS_OFF */
    NEMADC_SCANDOUBLE =  1U << 1,  /**< SCANDOUBLE */
    NEMADC_TESTMODE   =  1U << 0,  /**< TESTMODE */
    NEMADC_P_RGB3     =  0U << 5,  /**< P_RGB3 */
    NEMADC_S_RGBX4    =  1U << 5,  /**< S_RGBX4 */
    NEMADC_S_RGB3     =  2U << 5,  /**< S_RGB3 */
    NEMADC_S_12BIT    =  3U << 5,  /**< S_12BIT */
    NEMADC_LVDS_ISP68 =  4U << 5,  /**< LVDS_ISP68 */
    NEMADC_LVDS_ISP8  =  5U << 5,  /**< LVDS_ISP8 */
    NEMADC_T_16BIT    =  6U << 5,  /**< T_16BIT */
    NEMADC_BT656      =  7U << 5,  /**< BT656 */
    NEMADC_JDIMIP     =  8U << 5,  /**< JDIMIP */
    NEMADC_LUT8       =  1U << 20  /**< LUT8 */
} nemadc_videomode_t;

// Configuration Check
//--------------------------------------------------------------------------
typedef enum {
    NEMADC_CFG_PALETTE      = 1U <<  0,  /**< Global Gamma enabled */
    NEMADC_CFG_FIXED_CURSOR = 1U <<  1,  /**< Fixed Cursor enabled */
    NEMADC_CFG_PROGR_CURSOR = 1U <<  2,  /**< Programmable Cursor enabled */
    NEMADC_CFG_DITHERING    = 1U <<  3,  /**< Dithering enabled */
    NEMADC_CFG_FORMAT       = 1U <<  4,  /**< Formatting enabled */
    NEMADC_CFG_HiQ_YUV      = 1U <<  5,  /**< High Quality YUV converted enabled */
    NEMADC_CFG_DBIB         = 1U <<  6,  /**< DBI Type-B interface enabled */
    NEMADC_CFG_YUVOUT       = 1U <<  7,  /**< RGB to YUV converted */

    NEMADC_CFG_L0_ENABLED   = 1U <<  8,  /**< Layer 0 enabled */
    NEMADC_CFG_L0_BLENDER   = 1U <<  9,  /**< Layer 0 has blender */
    NEMADC_CFG_L0_SCALER    = 1U << 10,  /**< Layer 0 has scaler */
    NEMADC_CFG_L0_GAMMA     = 1U << 11,  /**< Layer 0 has gamma LUT */

    NEMADC_CFG_L1_ENABLED   = 1U << 12,  /**< Layer 1 enabled */
    NEMADC_CFG_L1_BLENDER   = 1U << 13,  /**< Layer 1 has blender */
    NEMADC_CFG_L1_SCALER    = 1U << 14,  /**< Layer 1 has scaler */
    NEMADC_CFG_L1_GAMMA     = 1U << 15,  /**< Layer 1 has gamma LUT */

    NEMADC_CFG_L2_ENABLED   = 1U << 16,  /**< Layer 2 enabled */
    NEMADC_CFG_L2_BLENDER   = 1U << 17,  /**< Layer 2 has blender */
    NEMADC_CFG_L2_SCALER    = 1U << 18,  /**< Layer 2 has scaler */
    NEMADC_CFG_L2_GAMMA     = 1U << 19,  /**< Layer 2 has gamma LUT */

    NEMADC_CFG_L3_ENABLED   = 1U << 20,  /**< Layer 3 enabled */
    NEMADC_CFG_L3_BLENDER   = 1U << 21,  /**< Layer 3 has blender */
    NEMADC_CFG_L3_SCALER    = 1U << 22,  /**< Layer 3 has scaler */
    NEMADC_CFG_L3_GAMMA     = 1U << 23,  /**< Layer 3 has gamma LUT */

    NEMADC_CFG_L4_ENABLED   = 1U << 24,
    NEMADC_CFG_L4_BLENDER   = 1U << 25,
    NEMADC_CFG_L4_SCALER    = 1U << 26,
    NEMADC_CFG_L4_GAMMA     = 1U << 27,

    NEMADC_CFG_L0_YUVMEM    = 1U << 28,
    NEMADC_CFG_L1_YUVMEM    = 1U << 29,
    NEMADC_CFG_L2_YUVMEM    = 1U << 30
} nemadc_config_t;
#define NEMADC_CFG_L3_YUVMEM  (1U << 31)
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------

//Display/LCD Parameters
//--------------------------------------------------------------------------
typedef struct __nemadc_display_t {
    uint32_t resx ;  /**< Resolution X */
    uint32_t resy ;  /**< Resolution Y */
    uint32_t fpx  ;  /**< Front Porch X */
    uint32_t fpy  ;  /**< Front Porch Y */
    uint32_t bpx  ;  /**< Back Porch X */
    uint32_t bpy  ;  /**< Back Porch Y */
    uint32_t blx  ;  /**< Blanking X */
    uint32_t bly  ;  /**< Blanking Y */
} nemadc_display_t;

//Layer Parameters
//--------------------------------------------------------------------------
typedef struct __nemadc_layer_t {
    void            *baseaddr_virt ; /**< Virtual Address */
    uintptr_t        baseaddr_phys ; /**< Physical Address */
    uint32_t         resx          ; /**< Resolution X */
    uint32_t         resy          ; /**< Resolution Y */
    int32_t          stride        ; /**< Stride */
    int32_t          startx        ; /**< Start X */
    int32_t          starty        ; /**< Start Y */
    uint32_t         sizex         ; /**< Size X */
    uint32_t         sizey         ; /**< Size Y */
    uint8_t          alpha         ; /**< Alpha */
    uint8_t          blendmode     ; /**< Blending Mode */
    uint8_t          buscfg        ; /**< ?? */
    nemadc_format_t  format        ; /**< Format */
    uint32_t         mode          ; /**< Mode */
    //-------------------
    uint32_t         u_base        ; /**< U Base */
    uint32_t         v_base        ; /**< Y Base */
    uint32_t         u_stride      ; /**< U Stride */
    uint32_t         v_stride      ; /**< V Stride */
    //-------------------
} nemadc_layer_t;

typedef enum {
    NEMADC_EN_PIXCLK          = (1U<<22),
    NEMADC_EN_CFCLK           = (1U<<23),
    NEMADC_EN_L0BUS           = (1U<<24),
    NEMADC_EN_L0PIX           = (1U<<25),
    NEMADC_EN_L1BUS           = (1U<<26),
    NEMADC_EN_L1PIX           = (1U<<27),
    NEMADC_EN_L2BUS           = (1U<<28),
    NEMADC_EN_L2PIX           = (1U<<29),
    NEMADC_EN_L3BUS           = (1U<<30),
} nemadc_clkctrl_t;
#define NEMADC_EN_L3PIX        (1U<<31)
//--------------------------------------------------------------------------

// Functions
//-----------------------------------------------------------------------------------------------------------------------
/** \brief Initialize NemaDC library
 *
 * \return -1 on error
 *
 */
int      nemadc_init       (void);

/** \brief Read Configuration Register
 *
 * \return Configuration Register Value
 *
 */
uint32_t nemadc_get_config(void);

/** \brief Read CRC Checksum Register
 *
 * \return CRC checksum value of last frame. For testing purposes
 *
 */
uint32_t nemadc_get_crc(void);

/** \brief Set NemaDC Background Color
 *
 * \param rgba Color as a 32-bit rgba value
 *
 */
void     nemadc_set_bgcolor(uint32_t rgba);

/** \brief Set Display timing parameters
 *
 * \param resx Resolution X
 * \param fpx Front Porch X
 * \param blx Blanking X
 * \param bpx Back Porch X
 * \param resy Resolution Y
 * \param fpy Front Porch Y
 * \param bly Blanking Y
 * \param bpy Back Porch Y
 *
 */
void     nemadc_timing(int resx, int fpx, int blx, int bpx,
                       int resy, int fpy, int bly, int bpy);

/** \brief Return stride size in bytes
 *
 * \param format Texture color format
 * \param format Texture width
 * \return Stride in bytes
 *
 */
int      nemadc_stride_size(nemadc_format_t format, int width);


/** \brief Set the built-in Clock Dividers and DMA Line Prefetch. (See Configuration Register 0x4)
 *
 * \param div Set Divider 1
 * \param div2 Set Divider 2
 * \param dma_prefetch Set number of lines for the dma to prefetch
 * \param phase Clock phase shift
 *
 */
void     nemadc_clkdiv(int div, int div2, int dma_prefetch, int phase);

/** \brief Control the clock gaters
 *
 * \param ctrl struct control
 */
void     nemadc_clkctrl   (nemadc_clkctrl_t ctrl);

/** \brief Set operation mode
 *
 * \param mode Mode of operation (See Register 0)
 */
void     nemadc_set_mode  (int mode);

/** \brief Get status from Status Register
 *
 *
 */
uint32_t nemadc_get_status (void);

/** \brief Request a VSync Interrupt without blocking
 *
 *
 */
void     nemadc_request_vsync_non_blocking(void);

/** \brief Set the Layer Mode. This function can enable a layer and set attributes to it
 *
 * \param layer_no The layer number
 * \param layer Layer Attributes struct
 *
 */
void     nemadc_set_layer (int layer_no, nemadc_layer_t *layer);

/** \brief Set the physical address of a layer.
 *
 * \param layer_no The layer number
 * \param addr Layer Physical Address
 *
 */
void     nemadc_set_layer_addr(int layer_no, uintptr_t addr);

/** \brief Set an entry in the lut8 Palette Gamma table for a layer
 *
 * \param layer Layer number
 * \param index Color Index
 * \param Color 32-bit RGBA color value or gamma index
 *
 */
void     nemadc_set_layer_gamma_lut(int layer, int index, int colour);


/** \brief Get an entry in the lut8 Palette Gamma table for a layer
 *
 * \param layer Layer number
 * \param index Color Index
 * \return Palette index
 *
 **/
int      nemadc_get_layer_gamma_lut(int layer, int index);

/** \brief Sets an entry in the lut8 Palatte Gamma table
 *
 * \param uint32_t index Color Index
 * \param uint32_t colour 32-bit RGBA colour value or Gamma index
 *
 **/
void     nemadc_set_palette(uint32_t index, uint32_t colour);

/** \brief Reads an entry from the lut8 Palatte Gamma table
 *
 * \param index Color Index
 * \return Return Colour for given palette index
 *
 **/
int      nemadc_get_palette(uint32_t index);

/** \brief Disable layer
 *
 * \param layer_no Layer Number
 *
 */
void     nemadc_layer_disable(int layer_no);

/** \brief Enable layer
 *
 * \param layer_no Layer Number
 *
 */
void     nemadc_layer_enable(int layer_no);

/** \brief Enable or Disable fixed cursor
 *
 * \param enable 1 for enable or 0 for disable cursor
 *
 */
void     nemadc_cursor_enable(int enable);

/** \brief Set the location of the cursor
 *
 * \param x Cursor X coordinate
 * \param x Cursor Y coordinate
 *
 */
void     nemadc_cursor_xy(int x, int y);


/** \brief Set programmable cursor image (32x32 pixels)
 *
 * \param img Base address of the 32x32 Cursor Image
 *
 */
void     nemadc_set_cursor_img(unsigned char *img);


/** \brief Set a color for the Cursor LUT
 *
 * \param index Color index
 * \param color 32-bit RGBA value
 *
 */
void     nemadc_set_cursor_lut(uint32_t index, uint32_t color);


/** \brief Check whether NemaDC supports a specific characteristic
 *
 * \param flag Flag to query
 * \return True if the characteristic is supported
 *
 */
unsigned char nemadc_check_config(nemadc_config_t flag);


/** \brief Read Color Mode Register
 *
 * \return Color mode register
 *
 */
uint32_t nemadc_get_col_mode(void);


/** \brief Get the number of layers available
 *
 * \return Number of layers
 *
 */
int      nemadc_get_layer_count(void);
//-----------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif
