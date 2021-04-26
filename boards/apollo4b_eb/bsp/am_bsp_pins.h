//*****************************************************************************
//
//  am_bsp_pins.h
//! @file
//!
//! @brief BSP pin configuration definitions.
//!
//! @addtogroup BSP Board Support Package (BSP)
//! @ingroup BSP
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

#ifndef AM_BSP_PINS_H
#define AM_BSP_PINS_H

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// COM_UART_TX pin: This pin is COM_UART transmit.
//
//*****************************************************************************
#define AM_BSP_GPIO_COM_UART_TX             53
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_COM_UART_TX;

//*****************************************************************************
//
// COM_UART_RX pin: This pin is COM_UART receive.
//
//*****************************************************************************
#define AM_BSP_GPIO_COM_UART_RX             55
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_COM_UART_RX;

//*****************************************************************************
//
// COM_UART_CTS pin: This pin is the COM_UART CTS.
//
//*****************************************************************************
#define AM_BSP_GPIO_COM_UART_CTS            59
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_COM_UART_CTS;

//*****************************************************************************
//
// COM_UART_RTS pin: This pin is the COM_UART RTS.
//
//*****************************************************************************
#define AM_BSP_GPIO_COM_UART_RTS            57
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_COM_UART_RTS;

//*****************************************************************************
//
// UART_TX pin: This pin is UART1 transmit.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART_TX                 12
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART_TX;

//*****************************************************************************
//
// UART_RX pin: This pin is UART1 receive.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART_RX                 14
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART_RX;

//*****************************************************************************
//
// UART_CTS pin: This pin is the UART1 CTS.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART_CTS                51
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART_CTS;

//*****************************************************************************
//
// UART_RTS pin: This pin is the UART1 RTS.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART_RTS                49
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART_RTS;

//*****************************************************************************
//
// IOM0_CS pin: I/O Master 0 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_CS                 50
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_CS;
#define AM_BSP_IOM0_CS_CHNL                 0

//*****************************************************************************
//
// IOM0_MISO pin: I/O Master 0 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_MISO               7
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_MISO;

//*****************************************************************************
//
// IOM0_MOSI pin: I/O Master 0 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_MOSI               6
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_MOSI;

//*****************************************************************************
//
// IOM0_SCK pin: I/O Master 0 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_SCK                5
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_SCK;

//*****************************************************************************
//
// IOM0_SCL pin: I/O Master 0 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_SCL                5
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_SCL;

//*****************************************************************************
//
// IOM0_SDA pin: I/O Master 0 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_SDA                6
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_SDA;

//*****************************************************************************
//
// IOM1_CS pin: I/O Master 1 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_CS                 51
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_CS;
#define AM_BSP_IOM1_CS_CHNL                 0

//*****************************************************************************
//
// IOM1_MISO pin: I/O Master 1 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_MISO               10
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_MISO;

//*****************************************************************************
//
// IOM1_MOSI pin: I/O Master 1 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_MOSI               9
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_MOSI;

//*****************************************************************************
//
// IOM1_SCK pin: I/O Master 1 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_SCK                8
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_SCK;

//*****************************************************************************
//
// IOM1_SCL pin: I/O Master 1 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_SCL                8
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_SCL;

//*****************************************************************************
//
// IOM1_SDA pin: I/O Master 1 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_SDA                9
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_SDA;

//*****************************************************************************
//
// IOM2_CS pin: I/O Master 2 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_CS                 84
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_CS;
#define AM_BSP_IOM2_CS_CHNL                 0

//*****************************************************************************
//
// IOM2_MISO pin: I/O Master 2 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_MISO               27
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_MISO;

//*****************************************************************************
//
// IOM2_MOSI pin: I/O Master 2 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_MOSI               26
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_MOSI;

//*****************************************************************************
//
// IOM2_SCK pin: I/O Master 2 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_SCK                25
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_SCK;

//*****************************************************************************
//
// IOM2_SCL pin: I/O Master 2 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_SCL                25
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_SCL;

//*****************************************************************************
//
// IOM2_SDA pin: I/O Master 2 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_SDA                26
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_SDA;

//*****************************************************************************
//
// IOM3_CS pin: I/O Master 3 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_CS                 85
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_CS;
#define AM_BSP_IOM3_CS_CHNL                 0

//*****************************************************************************
//
// IOM3_MISO pin: I/O Master 3 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_MISO               33
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_MISO;

//*****************************************************************************
//
// IOM3_MOSI pin: I/O Master 3 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_MOSI               32
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_MOSI;

//*****************************************************************************
//
// IOM3_SCK pin: I/O Master 3 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_SCK                31
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_SCK;

//*****************************************************************************
//
// IOM3_SCL pin: I/O Master 3 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_SCL                31
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_SCL;

//*****************************************************************************
//
// IOM3_SDA pin: I/O Master 3 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_SDA                32
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_SDA;

//*****************************************************************************
//
// IOM4_CS pin: I/O Master 4 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_CS                 43
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM4_CS;
#define AM_BSP_IOM4_CS_CHNL                 0

//*****************************************************************************
//
// IOM4_MISO pin: I/O Master 4 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_MISO               36
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM4_MISO;

//*****************************************************************************
//
// IOM4_MOSI pin: I/O Master 4 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_MOSI               35
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM4_MOSI;

//*****************************************************************************
//
// IOM4_SCK pin: I/O Master 4 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_SCK                34
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM4_SCK;

//*****************************************************************************
//
// IOM4_SCL pin: I/O Master 4 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_SCL                34
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM4_SCL;

//*****************************************************************************
//
// IOM4_SDA pin: I/O Master 4 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_SDA                35
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM4_SDA;

//*****************************************************************************
//
// IOM5_CS pin: I/O Master 5 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_CS                 60
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_CS;
#define AM_BSP_IOM5_CS_CHNL                 0

//*****************************************************************************
//
// IOM5_MISO pin: I/O Master 5 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_MISO               49
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_MISO;

//*****************************************************************************
//
// IOM5_MOSI pin: I/O Master 5 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_MOSI               48
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_MOSI;

//*****************************************************************************
//
// IOM5_SCK pin: I/O Master 5 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_SCK                47
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_SCK;

//*****************************************************************************
//
// IOM5_SCL pin: I/O Master 5 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_SCL                47
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_SCL;

//*****************************************************************************
//
// IOM5_SDA pin: I/O Master 5 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_SDA                48
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_SDA;

//*****************************************************************************
//
// IOM6_CS pin: I/O Master 6 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_CS                 30
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_CS;
#define AM_BSP_IOM6_CS_CHNL                 0

//*****************************************************************************
//
// IOM6_MISO pin: I/O Master 6 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_MISO               63
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_MISO;

//*****************************************************************************
//
// IOM6_MOSI pin: I/O Master 6 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_MOSI               62
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_MOSI;

//*****************************************************************************
//
// IOM6_SCK pin: I/O Master 6 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_SCK                61
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_SCK;

//*****************************************************************************
//
// IOM6_SCL pin: I/O Master 6 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_SCL                61
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_SCL;

//*****************************************************************************
//
// IOM6_SDA pin: I/O Master 6 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_SDA                62
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_SDA;

//*****************************************************************************
//
// IOM7_CS pin: I/O Master 7 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_CS                 88
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_CS;
#define AM_BSP_IOM7_CS_CHNL                 0

//*****************************************************************************
//
// IOM7_MISO pin: I/O Master 7 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_MISO               24
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_MISO;

//*****************************************************************************
//
// IOM7_MOSI pin: I/O Master 7 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_MOSI               23
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_MOSI;

//*****************************************************************************
//
// IOM7_SCK pin: I/O Master 7 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_SCK                22
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_SCK;

//*****************************************************************************
//
// IOM7_SCL pin: I/O Master 7 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_SCL                22
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_SCL;

//*****************************************************************************
//
// IOM7_SDA pin: I/O Master 7 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_SDA                23
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_SDA;

//*****************************************************************************
//
// IOS_CE pin: I/O Slave chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_CE                  3
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_CE;

//*****************************************************************************
//
// IOS_MISO pin: I/O Slave SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_MISO                2
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_MISO;

//*****************************************************************************
//
// IOS_MOSI pin: I/O Slave SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_MOSI                1
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_MOSI;

//*****************************************************************************
//
// IOS_SCK pin: I/O Slave SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_SCK                 0
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_SCK;

//*****************************************************************************
//
// IOS_SCL pin: I/O Slave I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_SCL                 0
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_SCL;

//*****************************************************************************
//
// IOS_SDA pin: I/O Slave I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_SDA                 1
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_SDA;

//*****************************************************************************
//
// IOS_INT pin: I/O Slave interrupt.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_INT                 4
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_INT;

//*****************************************************************************
//
// MSPI0_DDR_CE0 pin: MSPI0 chip select for 1.8V Octal DDR PSRAM
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_DDR_CE0           85
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_DDR_CE0;
#define AM_BSP_MSPI0_DDR_CE0_CHNL           0

//*****************************************************************************
//
// MSPI0_DDR_CE1 pin: MSPI0 chip select for 1.8V Octal DDR PSRAM
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_DDR_CE1           85
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_DDR_CE1;
#define AM_BSP_MSPI0_DDR_CE1_CHNL           1

//*****************************************************************************
//
// MSPI0_CE0 pin: MSPI0 chip select for 1.8V Octal Flash
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_CE0               73
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_CE0;
#define AM_BSP_MSPI0_CE0_CHNL               0

//*****************************************************************************
//
// MSPI0_CE1 pin: MSPI0 chip select for 1.8V PSRAM
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_CE1               52
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_CE1;
#define AM_BSP_MSPI0_CE1_CHNL               1

//*****************************************************************************
//
// MSPI0_D0 pin: MSPI0 data 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D0                64
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D0;

//*****************************************************************************
//
// MSPI0_D1 pin: MSPI0 data 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D1                65
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D1;

//*****************************************************************************
//
// MSPI0_D2 pin: MSPI0 data 2.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D2                66
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D2;

//*****************************************************************************
//
// MSPI0_D3 pin: MSPI0 data 3.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D3                67
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D3;

//*****************************************************************************
//
// MSPI0_D4 pin: MSPI0 data 4.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D4                68
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D4;

//*****************************************************************************
//
// MSPI0_D5 pin: MSPI0 data 5.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D5                69
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D5;

//*****************************************************************************
//
// MSPI0_D6 pin: MSPI0 data 6.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D6                70
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D6;

//*****************************************************************************
//
// MSPI0_D7 pin: MSPI0 data 7.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D7                71
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D7;

//*****************************************************************************
//
// MSPI0_SCK pin: MSPI0 clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_SCK               72
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_SCK;

//*****************************************************************************
//
// MSPI0_DQSDM pin: MSPI0 DQS DM.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_DQSDM             73
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_DQSDM;

//*****************************************************************************
//
// MSPI2_DDR_CE0 pin: MSPI2 chip select for 1.8V Octal DDR PSRAM
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_DDR_CE0           84
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_DDR_CE0;
#define AM_BSP_MSPI2_DDR_CE0_CHNL           0

//*****************************************************************************
//
// MSPI2_DDR_CE1 pin: MSPI2 chip select for 1.8V Octal DDR PSRAM
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_DDR_CE1           84
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_DDR_CE1;
#define AM_BSP_MSPI2_DDR_CE1_CHNL           1

//*****************************************************************************
//
// MSPI2_CE0 pin: MSPI2 chip select for 1.8V Octal Flash
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_CE0               83
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_CE0;
#define AM_BSP_MSPI2_CE0_CHNL               0

//*****************************************************************************
//
// MSPI2_CE1 pin: MSPI2 chip select for 1.8V PSRAM
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_CE1               86
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_CE1;
#define AM_BSP_MSPI2_CE1_CHNL               1

//*****************************************************************************
//
// MSPI2_D0 pin: MSPI2 data 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D0                74
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D0;

//*****************************************************************************
//
// MSPI2_D1 pin: MSPI2 data 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D1                75
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D1;

//*****************************************************************************
//
// MSPI2_D2 pin: MSPI2 data 2.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D2                76
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D2;

//*****************************************************************************
//
// MSPI2_D3 pin: MSPI2 data 3.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D3                77
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D3;

//*****************************************************************************
//
// MSPI2_D4 pin: MSPI2 data 4.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D4                78
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D4;

//*****************************************************************************
//
// MSPI2_D5 pin: MSPI2 data 5.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D5                79
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D5;

//*****************************************************************************
//
// MSPI2_D6 pin: MSPI2 data 6.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D6                80
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D6;

//*****************************************************************************
//
// MSPI2_D7 pin: MSPI2 data 7.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D7                81
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D7;

//*****************************************************************************
//
// MSPI2_SCK pin: MSPI2 clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_SCK               82
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_SCK;

//*****************************************************************************
//
// MSPI2_DQSDM pin: MSPI2 DQS DM.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_DQSDM             83
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_DQSDM;

//*****************************************************************************
//
// DISP_SPI_CS pin: Display SPI CS.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_SPI_CS             83
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_SPI_CS;

//*****************************************************************************
//
// DISP_SPI_SD pin: Display SPI Data Out.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_SPI_SD             74
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_SPI_SD;

//*****************************************************************************
//
// DISP_SPI_DCX pin: Display SPI DCx.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_SPI_DCX            75
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_SPI_DCX;

//*****************************************************************************
//
// DISP_SPI_SCK pin: Display SPI Clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_SPI_SCK            78
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_SPI_SCK;

//*****************************************************************************
//
// DISP_SPI_RES pin: Display SPI Reset.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_SPI_RES            4
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_SPI_RES;

//*****************************************************************************
//
// DISP_SPI_TE pin: Display SPI TE.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_SPI_TE             87
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_SPI_TE;

//*****************************************************************************
//
// DISP_DSPI_CS pin: Display DSPI CS.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_DSPI_CS            83
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_DSPI_CS;

//*****************************************************************************
//
// DISP_DSPI_D0 pin: Display DSPI Data Out 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_DSPI_D0            74
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_DSPI_D0;

//*****************************************************************************
//
// DISP_DSPI_D1 pin: Display DSPI Data Out 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_DSPI_D1            75
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_DSPI_D1;

//*****************************************************************************
//
// DISP_DSPI_SCK pin: Display DSPI Clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_DSPI_SCK           78
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_DSPI_SCK;

//*****************************************************************************
//
// DISP_DSPI_RES pin: Display DSPI Reset.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_DSPI_RES           4
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_DSPI_RES;

//*****************************************************************************
//
// DISP_DSPI_OLEDEN pin: Display DSPI OLED Enable.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_DSPI_OLEDEN        29
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_DSPI_OLEDEN;

//*****************************************************************************
//
// DISP_DSPI_TE pin: Display DSPI TE.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_DSPI_TE            87
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_DSPI_TE;

//*****************************************************************************
//
// DISP_QSPI_CS pin: Display QSPI CS.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_QSPI_CS            83
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_QSPI_CS;

//*****************************************************************************
//
// DISP_QSPI_D0 pin: Display QSPI Data Out 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_QSPI_D0            74
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_QSPI_D0;

//*****************************************************************************
//
// DISP_QSPI_D1 pin: Display QSPI Data Out 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_QSPI_D1            75
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_QSPI_D1;

//*****************************************************************************
//
// DISP_QSPI_D2 pin: Display QSPI Data Out 2.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_QSPI_D2            76
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_QSPI_D2;

//*****************************************************************************
//
// DISP_QSPI_D3 pin: Display QSPI Data Out 3.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_QSPI_D3            77
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_QSPI_D3;

//*****************************************************************************
//
// DISP_QSPI_SCK pin: Display QSPI Clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_QSPI_SCK           78
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_QSPI_SCK;

//*****************************************************************************
//
// DISP_QSPI_RES pin: Display QSPI Reset.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_QSPI_RES           4
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_QSPI_RES;

//*****************************************************************************
//
// DISP_QSPI_OLEDEN pin: Display QSPI OLED Enable.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_QSPI_OLEDEN        29
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_QSPI_OLEDEN;

//*****************************************************************************
//
// DISP_QSPI_TE pin: Display QSPI TE.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_QSPI_TE            87
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_QSPI_TE;

//*****************************************************************************
//
// SDIO_CMD pin: SDIO Command Line.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIO_CMD                83
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIO_CMD;

//*****************************************************************************
//
// SDIO_DAT0 pin: SDIO DAT0 Line.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIO_DAT0               84
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIO_DAT0;

//*****************************************************************************
//
// SDIO_DAT1 pin: SDIO DAT1 Line.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIO_DAT1               85
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIO_DAT1;

//*****************************************************************************
//
// SDIO_DAT2 pin: SDIO DAT2 Line.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIO_DAT2               86
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIO_DAT2;

//*****************************************************************************
//
// SDIO_DAT3 pin: SDIO DAT3 Line.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIO_DAT3               87
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIO_DAT3;

//*****************************************************************************
//
// SDIO_DAT4 pin: SDIO DAT4 Line.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIO_DAT4               79
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIO_DAT4;

//*****************************************************************************
//
// SDIO_DAT5 pin: SDIO DAT5 Line.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIO_DAT5               80
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIO_DAT5;

//*****************************************************************************
//
// SDIO_DAT6 pin: SDIO DAT6 Line.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIO_DAT6               81
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIO_DAT6;

//*****************************************************************************
//
// SDIO_DAT7 pin: SDIO DAT7 Line.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIO_DAT7               82
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIO_DAT7;

//*****************************************************************************
//
// SDIO_CLK pin: SDIO CLK Line.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIO_CLK                88
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIO_CLK;

//*****************************************************************************
//
// DISP_DSI_RES pin: Display DSI Reset.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_DSI_RES            4
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_DSI_RES;

//*****************************************************************************
//
// DISP_DSI_TE pin: Display DSI TE.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISP_DSI_TE             87
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_DISP_DSI_TE;

//*****************************************************************************
//
// ITM_SWO pin: ITM Serial Wire Output.
//
//*****************************************************************************
#define AM_BSP_GPIO_ITM_SWO                 28
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_ITM_SWO;

//*****************************************************************************
//
// VDD18_SWITCH pin: DSI power enable.
//
//*****************************************************************************
#define AM_BSP_GPIO_VDD18_SWITCH            95
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_VDD18_SWITCH;

#ifdef __cplusplus
}
#endif

#endif // AM_BSP_PINS_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
