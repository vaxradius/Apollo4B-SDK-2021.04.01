//*****************************************************************************
//
//! @file hci_drv_cooper.h
//!
//! @brief Support functions for the AMBIQ BTLE radio.
//
//*****************************************************************************
#include <stdbool.h>
#include "wsf_os_int.h"
#include "wsf_os.h"
#include "am_devices_cooper.h"

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
#ifndef HCI_DRV_COOPER_H
#define HCI_DRV_COOPER_H

//*****************************************************************************
//
// AMBIQ vendor specific events
//
//*****************************************************************************
// Tx power level in dBm.
typedef enum
{
    TX_POWER_LEVEL_MINUS_20P0_dBm,
    TX_POWER_LEVEL_MINUS_15P0_dBm,
    TX_POWER_LEVEL_MINUS_10P0_dBm,
    TX_POWER_LEVEL_MINUS_5P0_dBm,
    TX_POWER_LEVEL_0P0_dBm,
    TX_POWER_LEVEL_PLUS_3P0_dBm,
    TX_POWER_LEVEL_PLUS_4P0_dBm,
    TX_POWER_LEVEL_PLUS_6P0_dBm,
    TX_POWER_LEVEL_INVALID,
}txPowerLevel_t;

// For FCC continous wave testing
#define PAYL_CONTINUOUS_WAVE        0x10
// For FCC continuous modulation testing
#define PAYL_CONTINUOUS_MODULATE    0x11

#define CHL_2402_INDEX    0        // low frequency
#define CHL_2440_INDEX    19       // medium frequency
#define CHL_2480_INDEX    39       // high frequency

// Signatures for the image downloads
typedef enum
{
    SBL_FLAG_DBG_ENABLE       = (int)0x38B75A0Du,
    SBL_INFO0_PATCH_AVAILABLE = (int)0xB35D18C9u,
    SBL_FLAG_IMAGE_AVAILABLE  = (int)0xC593876Au,
}sbl_flag_e;

typedef enum
{
    HCI_DBG_RD_MEM_CMD_OPCODE                      = 0xFC01,
    HCI_DBG_WR_MEM_CMD_OPCODE                      = 0xFC02,

    HCI_DBG_ID_FLASH_CMD_OPCODE                    = 0xFC05,
    HCI_DBG_ER_FLASH_CMD_OPCODE                    = 0xFC06,
    HCI_DBG_WR_FLASH_CMD_OPCODE                    = 0xFC07,
    HCI_DBG_RD_FLASH_CMD_OPCODE                    = 0xFC08,

    HCI_DBG_PLF_RESET_CMD_OPCODE                   = 0xFC11,

    HCI_DBG_REG_RD_CMD_OPCODE                   = 0xFC39,
    HCI_DBG_REG_WR_CMD_OPCODE                   = 0xFC3A,

    // Ambiq Vendor Specific Command set Tx power level
    HCI_VSC_SET_TX_POWER_LEVEL      = 0xFC70,
    // Ambiq Vendor Specific Command  start transmitter test
    HCI_VSC_START_TRANS_TEST        = 0xFC71,
    // Ambiq Vendor Specific Command end transmitter test
    HCI_VSC_END_TRANS_TEST          = 0xFC72,
    // Ambiq Vendor Specific Command set debug log bitmap
    HCI_VSC_SET_LOG_BITMAP          = 0xFC73,
    // Ambiq Vendor Specific Command update bd address
    HCI_VSC_SET_BD_ADDR             = 0xFC74,
    // Ambiq Vendor Specific Command update FW
    HCI_VSC_UPDATE_FW               = 0xFC75,
    // Ambiq Vendor Specific Command get device ID
    HCI_VSC_GET_DEVICE_ID           = 0xFC76,
    // Ambiq Vendor Specific Command set NVDS parameters
    HCI_VSC_UPDATE_NVDS             = 0xFC77,
    // Ambiq Vendor Specific Command set link layer features
    HCI_VSC_UPDATE_LL_FEATURE        = 0xFC78,
}vsc_opcode;

#define MAX_MEM_ACCESS_SIZE   128
#define MAX_FLASH_ACCESS_SIZE   128

typedef enum
{
    RD_8_Bit                   = 8,
    /// 16 bit access types
    RD_16_Bit                  = 16,
    /// 32 bit access types
    RD_32_Bit                  = 32

}eMemAccess_type;

typedef enum
{
    /// PLATFORM RESET REASON: Reset and load FW from flash
    PLATFORM_RESET_TO_FW        = 0,
    /// PLATFORM RESET REASON: Reset and stay in ROM code
    PLATFORM_RESET_TO_ROM       = 1,
}ePlfResetReason_type;

/*! read memory variable command */
typedef struct
{
    ///Start address to read
    uint32_t start_addr;
    ///Access size
    uint8_t type;
    ///Length to read
    uint8_t length;
}hciRdMemCmd_t;


/*! write memory variable command */
typedef struct
{
    ///Start address to read
    uint32_t start_addr;
    ///Access size
    uint8_t type;
    ///Length to write
    uint8_t length;
    uint8_t data[MAX_MEM_ACCESS_SIZE];
}hciWrMemCmd_t;

typedef struct
{
    ///Flash type
    uint8_t flashtype;
    ///Start offset address
    uint32_t startoffset;
    ///Size to erase
    uint32_t size;
}hciErFlashCmd_t;

typedef struct
{
    ///Flash type
    uint8_t flashtype;
    ///Start offset address
    uint32_t startoffset;
    uint8_t length;
    uint8_t data[MAX_FLASH_ACCESS_SIZE];
}hciWrFlashCmd_t;

typedef struct
{
    ///Flash type
    uint8_t flashtype;
    ///Start offset address
    uint32_t startoffset;
    ///Size to read
    uint8_t size;
}hciRdFlashCmd_t;

typedef struct
{
    /// register address
    uint32_t addr;
}hciRegRdCmd_t;

typedef struct
{
    /// register address
    uint32_t addr;
    /// register value
    uint32_t value;
}hciRegWrCmd_t;

typedef struct
{
    /// reason
    uint8_t reason;
}hciPlfResetCmd_t;

#define NVDS_DATA_LEN    0xF0//0x00be
enum PARAM_ID
{
    /// Definition of the tag associated to each parameters
    /// Local Bd Address
    PARAM_ID_BD_ADDRESS                 = 0x01,
    /// Device Name
    PARAM_ID_DEVICE_NAME                = 0x02,
    /// 32K source
    PARAM_ID_32K_CLK_SOURCE             = 0x03,
    /// Radio Drift
    PARAM_ID_LPCLK_DRIFT                = 0x07,
    /// Radio Jitter
    PARAM_ID_LPCLK_JITTER               = 0x08,
    /// External wake-up time
    PARAM_ID_EXT_WAKEUP_TIME            = 0x0D,
    /// Oscillator wake-up time
    PARAM_ID_OSC_WAKEUP_TIME            = 0x0E,
    /// Radio wake-up time
    PARAM_ID_RM_WAKEUP_TIME             = 0x0F,
    /// UART baudrate
    PARAM_ID_UART_BAUDRATE              = 0x10,
    /// Enable sleep mode
    PARAM_ID_SLEEP_ENABLE               = 0x11,
    /// Enable External Wakeup
    PARAM_ID_EXT_WAKEUP_ENABLE          = 0x12,
    /// SP Private Key 192
    PARAM_ID_SP_PRIVATE_KEY_P192        = 0x13,
    /// SP Public Key 192
    PARAM_ID_SP_PUBLIC_KEY_P192         = 0x14,

    /// Activity Move Configuration (enables/disables activity move for BLE connections and BT (e)SCO links)
    PARAM_ID_ACTIVITY_MOVE_CONFIG       = 0x15,

    /// Enable/disable scanning for extended advertising PDUs
    PARAM_ID_SCAN_EXT_ADV               = 0x16,

    /// Duration of the schedule reservation for long activities such as scan, inquiry, page, HDC advertising
    PARAM_ID_SCHED_SCAN_DUR             = 0x17,

    /// Programming delay, margin for programming the baseband in advance of each activity (in half-slots)
    PARAM_ID_PROG_DELAY                 = 0x18,

    /// Enable/disable channel assessment for BT and/or BLE
    PARAM_ID_CH_ASS_EN                  = 0x19,

    /// Synchronous links configuration
    PARAM_ID_SYNC_CONFIG                = 0x2C,
    /// PCM Settings
    PARAM_ID_PCM_SETTINGS               = 0x2D,
    /// Sleep algorithm duration
    PARAM_ID_SLEEP_ALGO_DUR             = 0x2E,
    /// Tracer configuration
    PARAM_ID_TRACER_CONFIG              = 0x2F,

    /// Diagport configuration
    PARAM_ID_DIAG_BT_HW                 = 0x30,
    PARAM_ID_DIAG_BLE_HW                = 0x31,
    PARAM_ID_DIAG_SW                    = 0x32,
    PARAM_ID_DIAG_DM_HW                 = 0x33,
    PARAM_ID_DIAG_PLF                   = 0x34,

    /// IDC selection (for audio demo)
    PARAM_ID_IDCSEL_PLF                 = 0x37,

    /// RSSI threshold tags
    PARAM_ID_RSSI_HIGH_THR              = 0x3A,
    PARAM_ID_RSSI_LOW_THR               = 0x3B,
    PARAM_ID_RSSI_INTERF_THR            = 0x3C,

    /// RF BTIPT
    PARAM_ID_RF_BTIPT_VERSION          = 0x3E,
    PARAM_ID_RF_BTIPT_XO_SETTING       = 0x3F,
    PARAM_ID_RF_BTIPT_GAIN_SETTING     = 0x40,


    PARAM_ID_BT_LINK_KEY_FIRST          = 0x60,
    PARAM_ID_BT_LINK_KEY_LAST           = 0x67,

    PARAM_ID_BLE_LINK_KEY_FIRST         = 0x70,
    PARAM_ID_BLE_LINK_KEY_LAST          = 0x7F,
    /// SC Private Key (Low Energy)
    PARAM_ID_LE_PRIVATE_KEY_P256        = 0x80,
    /// SC Public Key (Low Energy)
    PARAM_ID_LE_PUBLIC_KEY_P256         = 0x81,
    /// SC Debug: Used Fixed Private Key from NVDS (Low Energy)
    PARAM_ID_LE_DBG_FIXED_P256_KEY      = 0x82,
    /// SP Private Key (classic BT)
    PARAM_ID_SP_PRIVATE_KEY_P256        = 0x83,
    /// SP Public Key (classic BT)
    PARAM_ID_SP_PUBLIC_KEY_P256         = 0x84,

    /// LE Coded PHY 500 Kbps selection
    PARAM_ID_LE_CODED_PHY_500           = 0x85,

    /// Application specific
    PARAM_ID_APP_SPECIFIC_FIRST         = 0x90,
    PARAM_ID_APP_SPECIFIC_LAST          = 0xAF,

    /// Mesh NVDS values
    PARAM_ID_MESH_SPECIFIC_FIRST        = 0xB0,
    PARAM_ID_MESH_SPECIFIC_LAST         = 0xF0,
};


#define LL_FEATURES_BYTE0  ( HCI_LE_SUP_FEAT_ENCRYPTION  \
                                 | HCI_LE_SUP_FEAT_CONN_PARAM_REQ_PROC \
                                 | HCI_LE_SUP_FEAT_EXT_REJECT_IND \
                                 | HCI_LE_SUP_FEAT_SLV_INIT_FEAT_EXCH \
                                 | HCI_LE_SUP_FEAT_LE_PING \
                                 | HCI_LE_SUP_FEAT_DATA_LEN_EXT \
                                 | HCI_LE_SUP_FEAT_PRIVACY \
                                 | HCI_LE_SUP_FEAT_EXT_SCAN_FILT_POLICY )

#define LL_FEATURES_BYTE1  ( HCI_LE_SUP_FEAT_LE_2M_PHY \
                             | HCI_LE_SUP_FEAT_LE_EXT_ADV \
                             | HCI_LE_SUP_FEAT_LE_PER_ADV \
                             | HCI_LE_SUP_FEAT_CH_SEL_2 )

#define LL_FEATURES_BYTE2  ( HCI_LE_SUP_FEAT_MIN_NUN_USED_CHAN \
                             | HCI_LE_SUP_FEAT_CONN_CTE_REQ \
                             | HCI_LE_SUP_FEAT_CONN_CTE_RSP \
                             | HCI_LE_SUP_FEAT_CONNLESS_CTE_TRANS \
                             | HCI_LE_SUP_FEAT_CONNLESS_CTE_RECV \
                             | HCI_LE_SUP_FEAT_ANTENNA_SWITCH_AOD \
                             | HCI_LE_SUP_FEAT_ANTENNA_SWITCH_AOA \
                             | HCI_LE_SUP_FEAT_RECV_CTE )

#define LL_FEATURES_BYTE3  (HCI_LE_SUP_FEAT_PAST_SENDER \
                             | HCI_LE_SUP_FEAT_PAST_RECIPIENT \
                             | HCI_LE_SUP_FEAT_SCA_UPDATE \
                             | HCI_LE_SUP_FEAT_REMOTE_PUB_KEY_VALIDATION  )


// NVDS parameter setting array

//NVDS_MAGIC_NUMBER
#define NVDS_PARAMETER_MAGIC_NUMBER     0x4e,0x56,0x44,0x53
//device name
#define NVDS_PARAMETER_DEVICE_NAME      PARAM_ID_DEVICE_NAME,0x06,0x06,0x43,0x6F,0x6F,0x70,0x65,0x72
//32K source
#define NVDS_PARAMETER_EXT_32K_CLK_SOURCE PARAM_ID_32K_CLK_SOURCE,0x06,0x01,0x01
//Radio Drift
#define NVDS_PARAMETER_LPCLK_DRIFT      PARAM_ID_LPCLK_DRIFT,0x06,0x2,0xF4,0x01
//External wake-up time
#define NVDS_PARAMETER_WAKEUP_TIME      PARAM_ID_EXT_WAKEUP_TIME,0x06,0x02,0x20,0x03
//Oscillator wake-up time
#define NVDS_PARAMETER_OSC_WAKEUP_TIME  PARAM_ID_OSC_WAKEUP_TIME,0x06,0x02,0x20,0x03
//Radio wake-up time
#define NVDS_PARAMETER_RM_WAKEUP_TIME   PARAM_ID_RM_WAKEUP_TIME,0x06,0x02,0x1E,0x00
//set UART_BAUDRATE
#define NVDS_PARAMETER_UART_BAUDRATE    PARAM_ID_UART_BAUDRATE,0x06,0x04,0x00,0x10,0x0E,0x00
//sleep algorithm enabled
#define NVDS_PARAMETER_SLEEP_ENABLE     PARAM_ID_SLEEP_ENABLE,0x06,0x01,0x01
//sleep algorithm disabled
#define NVDS_PARAMETER_SLEEP_DISABLE    PARAM_ID_SLEEP_ENABLE,0x06,0x01,0x00
//external wake-up support
#define NVDS_PARAMETER_EXT_WAKEUP_ENABLE PARAM_ID_EXT_WAKEUP_ENABLE,0x06,0x01,0x01
//Activity Move Configuration
#define NVDS_PARAMETER_ACTIVITY_MOVE_CONFIG PARAM_ID_ACTIVITY_MOVE_CONFIG,0x06,0x01,0x01
//Enable/disable scanning for extended advertising PDUs
#define NVDS_PARAMETER_SCAN_EXT_ADV      PARAM_ID_SCAN_EXT_ADV,0x06,0x01,0x01
//default scan duration
#define NVDS_PARAMETER_SCHED_SCAN_DUR    PARAM_ID_SCHED_SCAN_DUR,0x06,0x02,0xE4,0x57
//Programming delay
#define NVDS_PARAMETER_PROG_DELAY        PARAM_ID_PROG_DELAY,0x06,0x01,0x1
//channel assessment for BT and/or BLE
#define NVDS_PARAMETER_CH_ASS_EN         PARAM_ID_CH_ASS_EN,0x06,0x01,0x1
// sleep algorithm duration
#define NVDS_PARAMETER_SLEEP_ALGO_DUR   PARAM_ID_SLEEP_ALGO_DUR,0x06,0x02,0xF4,0x01
// debug trace config
#define NVDS_PARAMETER_TRACE_CONFIG     PARAM_ID_TRACER_CONFIG,0x06,0x04,0x00,0x00,0x00,0x00
// diagnostic port
#define NVDS_PARAMETER_DIAG_BLE_HW      PARAM_ID_DIAG_BLE_HW,0x06,0x04,0x00,0x00,0x00,0x00
// SW diags configuration
#define NVDS_PARAMETER_DIAG_SW          PARAM_ID_DIAG_SW,0x06,0x04,0xFF,0xFF,0xFF,0xFF
// diagport configuration
#define NVDS_PARAMETER_DIAG_DM_HW       PARAM_ID_DIAG_DM_HW,0x06,0x04,0x00,0x00,0x00,0x00
//SC Private Key (Low Energy)
#define NVDS_PARAMETER_LE_PRIVATE_KEY_P256   PARAM_ID_LE_PRIVATE_KEY_P256,0x06,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, \
                                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, \
                                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
// SC Debug: Used Fixed Private Key from NVDS (Low Energy)
#define NVDS_PARAMETER_LE_DBG_FIXED_P256_KEY  PARAM_ID_LE_DBG_FIXED_P256_KEY,0x06,0x01,0x00
// LE Coded PHY 500 Kbps selection
#define NVDS_PARAMETER_LE_CODED_PHY_500   PARAM_ID_LE_CODED_PHY_500,0x06,0x01,0x00

#define MIN_SWITCHING_PATTERN_LEN  (0x02)
#define TEST_LEN_DEFAULT        (0x25)

//*****************************************************************************
//
// Hci driver functions unique to Cooper
//
//*****************************************************************************
extern void HciDrvHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);
extern void HciDrvHandlerInit(wsfHandlerId_t handlerId);
extern bool HciVscSetRfPowerLevelEx(txPowerLevel_t txPowerlevel);
extern void HciVscUpdateFw(sbl_flag_e update_fw);


extern bool HciVscReadMem(uint32_t start_addr, eMemAccess_type size,uint8_t length);
extern bool HciVscWriteMem(uint32_t start_addr, eMemAccess_type size,uint8_t length, uint8_t *data);
extern void HciVscGetFlashId(void);
extern void HciVscEraseFlash(uint8_t type, uint32_t offset,uint32_t size);
extern bool HciVscWriteFlash(uint8_t type, uint32_t offset,uint32_t length, uint8_t *data);
extern bool HciVscReadFlash(uint8_t type, uint32_t offset,uint32_t size);
extern void HciVscReadReg(uint32_t reg_addr);
extern void HciVscWriteReg(uint32_t reg_addr, uint32_t value);
extern void HciVscPlfReset(ePlfResetReason_type reason);
extern void HciVscUpdateBDAddress(void);
extern bool_t HciVscSetCustom_BDAddr(uint8_t *bd_addr);
void HciVscUpdateNvdsParam(void);
void HciVscUpdateLinklayerFeature(void);


#endif // HCI_DRV_COOPER_H
