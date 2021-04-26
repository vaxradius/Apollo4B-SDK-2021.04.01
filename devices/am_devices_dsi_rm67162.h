//*****************************************************************************
//
//! @file am_devices_dsi_rm67162.c
//!
//! @brief Generic RM67162 DSI driver.
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


#ifndef AM_DEVICES_DSI_RM67162_H
#define AM_DEVICES_DSI_RM67162_H

#include "stdint.h"
#include "nema_dc.h"
#include "nema_dc_hal.h"
#include "nema_dc_mipi.h"
#include "nema_dc_regs.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Global definitions for the commands
//
//*****************************************************************************
#define TB_LCDPANEL_MIPI_DBIB      14

//*****************************************************************************
//
// Global type definitions.
//
//*****************************************************************************
typedef enum
{
    DC_dt_vsync_start                        = 0x01,
    DC_dt_vsync_end                          = 0x11,
    DC_dt_hsync_start                        = 0x21,
    DC_dt_hsync_end                          = 0x31,
    DC_dt_cmpr_mode                          = 0x07,
    DC_dt_end_of_trans                       = 0x08,
    DC_dt_pic_param                          = 0x0a,
    DC_dt_cmpr_pix_stream                    = 0x0b,
    DC_dt_color_mode_off                     = 0x02,
    DC_dt_color_mode_on                      = 0x12,
    DC_dt_shut_down_peripheral               = 0x22,
    DC_dt_turn_on_peripheral                 = 0x32,
    DC_dt_generic_short_write_param_no       = 0x03,
    DC_dt_generic_short_write_param_n1       = 0x13,
    DC_dt_generic_short_write_param_n2       = 0x23,
    DC_dt_generic_read_param_no              = 0x04,
    DC_dt_generic_read_param_n1              = 0x14,
    DC_dt_execute_queue                      = 0x16,
    DC_dt_generic_read_param_n2              = 0x24,
    DC_dt_DCS_short_write_param_no           = 0x05,
    DC_dt_DCS_short_write_param_n1           = 0x15,
    DC_dt_DCS_read_param_no                  = 0x06,
    DC_dt_set_max_return_packet_size         = 0x37,
    DC_dt_blanking_packet                    = 0x19,
    DC_dt_generic_long_write                 = 0x29,
    DC_dt_DCS_long_write                     = 0x39,
    DC_dt_packed_pixel_stream_rgb565         = 0x0e,
    DC_dt_packed_pixel_stream_rgb666         = 0x1e,
    DC_dt_loosely_packed_pixel_stream_rgb666 = 0x2e,
    DC_dt_loosely_packed_pixel_stream_rgb888 = 0x3e,

    NemaDC_dcs_datacmd                   = 0U,
    NemaDC_ge_data                       = (1U << 30),
    NemaDC_ge_cmd                        = (1 << 31),
    NemaDC_ge_datacmd                    = ((1 << 30) | (1 << 31))
} dc_ext_data_types_t;

typedef enum
{
    NemaDC_snapshot              = 0xff,

    NemaDC_store_base_addr       = (1  << 31),
    NemaDC_DBI_cmd               = (1U << 30),
    NemaDC_wcmd16                = (1U << 28),
    NemaDC_wcmd24                = (1U << 29),
    NemaDC_rcmd16                = (1U << 28),
    NemaDC_rcmd24                = (1U << 29),
    NemaDC_rcmd32                = (1U << 29) | (1U << 28),
    NemaDC_mask_qspi             = (1U << 27),
    NemaDC_DBI_ge                = (1U << 27),
    NemaDC_DBI_read              = (1U << 26),
    NemaDC_ext_ctrl              = (1U << 25),
    NemaDC_sline_cmd             = (1U << 24),

    NemaDC_read_byte             = (0U << 30),
    NemaDC_read_2byte            = (1U << 30),
    NemaDC_read_3byte            = (2  << 30),
    NemaDC_read_4byte            = (3  << 30)

} dc_mipi_cmd_t;

typedef enum
{
    DC_STATUS_rsrvd_0            = (1  << 31),
    DC_STATUS_rsrvd_1            = (1U << 30),
    DC_STATUS_rsrvd_2            = (1U << 29),
    DC_STATUS_rsrvd_3            = (1U << 28),
    DC_STATUS_rsrvd_4            = (1U << 27),
    DC_STATUS_rsrvd_5            = (1U << 26),
    DC_STATUS_rsrvd_6            = (1U << 25),
    DC_STATUS_rsrvd_7            = (1U << 24),
    DC_STATUS_rsrvd_8            = (1U << 23),
    DC_STATUS_rsrvd_9            = (1U << 22),
    DC_STATUS_rsrvd_10           = (1U << 21),
    DC_STATUS_rsrvd_11           = (1U << 20),
    DC_STATUS_rsrvd_12           = (1U << 19),
    DC_STATUS_rsrvd_13           = (1U << 18),
    DC_STATUS_rsrvd_14           = (1U << 17),
    DC_STATUS_rsrvd_15           = (1U << 16),
    DC_STATUS_dbi_cmd_ready      = (1U << 16), // DBI i/f fifo full
    DC_STATUS_dbi_cs             = (1U << 14), // DBI/SPI i/f active transaction
    DC_STATUS_frame_end          = (1U << 13), // End of frame pulse
    DC_STATUS_dbi_pending_trans  = (1U << 12), // pending command/data transaction
    DC_STATUS_dbi_pending_cmd    = (1U << 11), // pending command
    DC_STATUS_dbi_pending_data   = (1U << 10), // pending pixel data
    DC_STATUS_dbi_busy           = ((1U << 14) | (1U << 13) | (1U << 12) | (1U << 11)), // DBI i/f busy
    DC_STATUS_mmu_error          = (1U << 9), // -- not implemented --
    DC_STATUS_te                 = (1U << 8), // tearing
    DC_STATUS_sticky             = (1U << 7), // underflow flag
    DC_STATUS_underflow          = (1U << 6), // underflow signal
    DC_STATUS_LASTROW            = (1U << 5), // last scan-row (?)
    DC_STATUS_DPI_Csync          = (1U << 4), // DPI C-sync
    DC_STATUS_vsync_te           = (1U << 3), // Vsync or Tearing
    DC_STATUS_hsync              = (1U << 2), // Hsync
    DC_STATUS_framegen_busy      = (1U << 1), // Frame-generation in-progress
    DC_STATUS_ACTIVE             = (1U << 0), // active
} dc_status_t;

typedef enum
{
    NemaDC_clkctrl_cg_l3_bus_clk =  (1  << 31),
    NemaDC_clkctrl_cg_l3_pix_clk =  (1U << 30),
    NemaDC_clkctrl_cg_l2_bus_clk =  (1U << 29),
    NemaDC_clkctrl_cg_l2_pix_clk =  (1U << 28),
    NemaDC_clkctrl_cg_l1_bus_clk =  (1U << 27),
    NemaDC_clkctrl_cg_l1_pix_clk =  (1U << 26),
    NemaDC_clkctrl_cg_l0_bus_clk =  (1U << 25),
    NemaDC_clkctrl_cg_l0_pix_clk =  (1U << 24),
    NemaDC_clkctrl_cg_regfil_clk =  (1U << 23),
    NemaDC_clkctrl_cg_bypass_clk =  (1U << 22),
    //---------------------------------------
    NemaDC_clkctrl_cg_rsrvd_21   =  (1U << 21),
    NemaDC_clkctrl_cg_rsrvd_20   =  (1U << 20),
    NemaDC_clkctrl_cg_rsrvd_19   =  (1U << 19),
    NemaDC_clkctrl_cg_rsrvd_18   =  (1U << 18),
    NemaDC_clkctrl_cg_rsrvd_17   =  (1U << 17),
    NemaDC_clkctrl_cg_rsrvd_16   =  (1U << 16),
    NemaDC_clkctrl_cg_rsrvd_15   =  (1U << 15),
    NemaDC_clkctrl_cg_rsrvd_14   =  (1U << 14),
    NemaDC_clkctrl_cg_rsrvd_13   =  (1U << 13),
    NemaDC_clkctrl_cg_rsrvd_12   =  (1U << 12),
    NemaDC_clkctrl_cg_rsrvd_11   =  (1U << 11),
    NemaDC_clkctrl_cg_rsrvd_10   =  (1U << 10),
    NemaDC_clkctrl_cg_rsrvd_9    =  (1U << 9),
    NemaDC_clkctrl_cg_rsrvd_8    =  (1U << 8),
    NemaDC_clkctrl_cg_rsrvd_7    =  (1U << 7),
    NemaDC_clkctrl_cg_rsrvd_6    =  (1U << 6),
    NemaDC_clkctrl_cg_rsrvd_5    =  (1U << 5),
    NemaDC_clkctrl_cg_rsrvd_4    =  (1U << 4),
    NemaDC_clkctrl_cg_rsrvd_3    =  (1U << 3),
    //---------------------------------------
    NemaDC_clkctrl_cg_clk_swap   =  (1U << 2),
    NemaDC_clkctrl_cg_clk_inv    =  (1U << 1),
    NemaDC_clkctrl_cg_clk_en     =  (1U << 0)
    //---------------------------------------
} dc_clkctrl_cg_t;

typedef enum
{
    FMT_RGB565 = 0,
    FMT_RGB888,
    FMT_NUM
} pixel_format_t;

typedef void (*CallbackFun)(void *contex);

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern uint32_t
am_devices_dsi_rm67162_init(uint32_t ui32PixelFormat, uint16_t ui16ResX, uint16_t ui16ResY, uint16_t ui16MinX, uint16_t ui16MinY);
extern void
dsi_send_frame_single_start(uint32_t ui32Mode);
extern void
dsi_send_frame_single_end(void);
extern void
dsi_send_frame_single(uint32_t ui32Mode);
extern uint32_t
am_devices_dsi_rm67162_set_region(uint16_t ui16ResX, uint16_t ui16ResY, uint16_t ui16MinX, uint16_t ui16MinY);
void
send_reset_signal(void);
void
dsi_dcs_write(uint8_t ui8Cmd, uint8_t* pui8Data, uint8_t ui8Len, bool bHS);
uint32_t
dsi_dcs_read(uint8_t cmd, uint8_t n_data, bool bHS);
void
dsi_generic_write(uint8_t* pui8Data, uint8_t ui8Len, bool bHS);
uint32_t
dsi_generic_read(uint8_t* cmd, uint8_t n_para, uint8_t n_data, bool bHS);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_DSI_RM67162_H

