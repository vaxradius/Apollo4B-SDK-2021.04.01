//*****************************************************************************
//
//! @file am_devices_nemadc_rm67162.c
//!
//! @brief Generic Raydium TFT display driver.
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


#ifndef AM_DEVICES_NEMADC_RM67162_H
#define AM_DEVICES_NEMADC_RM67162_H

#define SPI_WRITE                                  (2U)
#define SPI_READ                                   (3U)

#define SPICMD                                 (1U << 5U)
#define QSPICMD                                (0U << 5U)

#define QSPIDATA                               (1U << 4U)

#define CMD_OFFSET                                 (8U)

#define CMD1_DATA1         (                 SPI_WRITE)
#define CMD1_DATA4         ( SPICMD | QSPIDATA | SPI_WRITE)
#define CMD4_DATA4           (QSPICMD | QSPIDATA | SPI_WRITE) )

#define SPI4_MODE 1
#define DSPI_MODE 2
#define QSPI_MODE 3

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
// External function definitions.
//
//*****************************************************************************
extern uint32_t
am_devices_nemadc_rm67162_init(uint32_t mode, uint32_t pixel_format, uint16_t resx, uint16_t resy, uint16_t minx, uint16_t miny);
extern uint32_t
am_devices_nemadc_rm67162_set_region(uint32_t mode, uint16_t resx, uint16_t resy, uint16_t minx, uint16_t miny);
extern void
nemadc_send_frame_single(void);
#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_RM67162_H

