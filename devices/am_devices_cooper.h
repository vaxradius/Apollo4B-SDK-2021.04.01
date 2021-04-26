//*****************************************************************************
//
//! @file am_devices_cooper.h
//!
//! @brief A re-implementation of the Apollo BLE interface using the IOM.
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

#ifndef AM_DEVICES_COOPER_H
#define AM_DEVICES_COOPER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "am_util.h"
#include <stdlib.h>
//*****************************************************************************
//
// Type definitions.
//
//*****************************************************************************
#define am_devices_cooper_buffer(A)                                                  \
    union                                                                     \
    {                                                                         \
        uint32_t words[(A + 3) >> 2];                                         \
        uint8_t bytes[A];                                                     \
    }

//*****************************************************************************
//
// Print Errors.
//
//*****************************************************************************
#define PRINT_ERRORS(x)                                                       \
    if (x)                                                                    \
    {                                                                         \
        am_util_debug_printf("%s. Line %d ERROR: 0x%08x\n",                   \
                             __FILE__, __LINE__, (x));                        \
        while (1);                                                            \
    }

#define WHILE_TIMEOUT_MS(expr, limit, timeout)                                    \
        {                                                                         \
            uint32_t ui32Timeout = 0;                                             \
            while (expr)                                                          \
            {                                                                     \
                if (ui32Timeout == (limit * 1000))                                \
                {                                                                 \
                    timeout = 1;                                                  \
                    break;                                                        \
                }                                                                 \
                am_util_delay_us(1);                                              \
                ui32Timeout++;                                                    \
            }                                                                     \
        }

typedef enum
{
    AM_DEVICES_COOPER_STATUS_SUCCESS,
    AM_DEVICES_COOPER_STATUS_ERROR,
    //
    // This error occurs when an HCI read or write function is called while
    // another HCI communication function is already in progress.
    //
    AM_DEVICES_COOPER_STATUS_BUS_BUSY,

    //
    // This error means that the MCU tried to execute an HCI write, but can not
    // get the space available (0xA868).
    // This might mean that the controller has not finished the wakeup process,
    // the MCU need to wait until the IRQ is asserted.
    //
    AM_DEVICES_COOPER_STATUS_CONTROLLER_NOT_READY,

    // Wrong length of data been read back (==0 or > 260)
    AM_DEVICES_COOPER_STATUS_WRONG_DATA_LENGTH,

    //
    // We are expecting an HCI response to a packet we just sent, but the BLE
    // core isn't asserting BLEIRQ. Its software may have crashed, and it may
    // need to restart.
    //
    AM_DEVICES_COOPER_STATUS_NO_RESPONSE,

    // The transaction ends up in error and cannot finish the HCI packet R/W
    AM_DEVICES_COOPER_STATUS_PACKET_INCOMPLETE,

    // The CQ does not complete transaction before times out
    AM_DEVICES_COOPER_STATUS_TIMEOUT,
} am_devices_cooper_status_t;

#define AM_DEVICES_COOPER_QFN_PART            0
#if (AM_DEVICES_COOPER_QFN_PART)
#define AM_DEVICES_COOPER_IRQ_PIN             73
#define AM_DEVICES_COOPER_RESET_PIN           57
#define AM_DEVICES_COOPER_CLKREQ_PIN          64
#define AM_DEVICES_COOPER_STATUS_PIN          66
#define AM_DEVICES_COOPER_32M_OSCEN_PIN       67
#define AM_DEVICES_COOPER_SPI_CS              72 // BGA&SIP share the same CS pin(NCE72) on the QFN shiled board
#define g_AM_DEVICES_COOPER_SPI_CS            g_AM_BSP_GPIO_IOM2_CS
#else
#define AM_DEVICES_COOPER_IRQ_PIN             39
#define AM_DEVICES_COOPER_CLKREQ_PIN          40
#define AM_DEVICES_COOPER_CLKACK_PIN          41
#define AM_DEVICES_COOPER_RESET_PIN           42
#define AM_DEVICES_COOPER_STATUS_PIN          44
#define AM_DEVICES_COOPER_SWDIO               97
#define AM_DEVICES_COOPER_SWCLK               98
#define AM_DEVICES_COOPER_32M_CLK             46
#define AM_DEVICES_COOPER_32K_CLK             45
#define AM_DEVICES_COOPER_SPI_CS              AM_BSP_GPIO_IOM4_CS
#define g_AM_DEVICES_COOPER_SPI_CS            g_AM_BSP_GPIO_IOM4_CS
#endif


//*****************************************************************************
//
// Configurable buffer sizes.
//
//*****************************************************************************
#define AM_DEVICES_COOPER_MAX_TX_PACKET       524 // the max packet of SBL to controller is 512 plus 12 bytes header
#define AM_DEVICES_COOPER_MAX_RX_PACKET       258 // 255 data + 3 header

//*****************************************************************************
//
// SPI configuration.
//
//*****************************************************************************
#if (AM_DEVICES_COOPER_QFN_PART)
#define SPI_MODULE           2
#define AM_COOPER_IRQn       GPIO0_405F_IRQn
#define am_cooper_irq_isr    am_gpio0_405f_isr
//
// we need to slow down SPI clock for fly-wire case in between
// Apollo3/3p/4 EB/EVBB and Cooper QFN part. 8MHz is chosen conservatively.
//
#define COOPER_IOM_FREQ         AM_HAL_IOM_8MHZ //TODO/Raise to 24M when the shiled board is ready
#else
#define SPI_MODULE           4
#define AM_COOPER_IRQn       GPIO0_203F_IRQn
#define am_cooper_irq_isr    am_gpio0_203f_isr
#define COOPER_IOM_FREQ         AM_HAL_IOM_24MHZ
#endif

//
// Take over the interrupt handler for whichever IOM we're using.
//
#define cooper_iom_isr                                                          \
    am_iom_isr1(SPI_MODULE)
#define am_iom_isr1(n)                                                        \
    am_iom_isr(n)
#define am_iom_isr(n)                                                         \
    am_iomaster ## n ## _isr

#define IOM_INTERRUPT1(n)       AM_HAL_INTERRUPT_IOMASTER ## n
#define IOM_INTERRUPT(n)        IOM_INTERRUPT1(n)
#define COOPER_IOM_IRQn         ((IRQn_Type)(IOMSTR0_IRQn + SPI_MODULE))

//*****************************************************************************
//
// Vendor Specific commands.
//
// Note: Lengths are reported as "4 + <parameter length>". Each vendor-specific
// header is 4 bytes long. This definition allows the macro version of the
// length to be used in all BLE APIs.
//
//*****************************************************************************
#define AM_DEVICES_COOPER_SET_BD_ADDR_OPCODE               0xFC74
#define AM_DEVICES_COOPER_SET_BD_ADDR_LENGTH               (4 + 6)

#define AM_DEVICES_COOPER_SET_TX_POWER_OPCODE              0xFC70
#define AM_DEVICES_COOPER_SET_TX_POWER_LENGTH              (4 + 1)

#define AM_DEVICES_COOPER_GET_DEVICE_ID_OPCODE             0xFC76
#define AM_DEVICES_COOPER_GET_DEVICE_ID_LENGTH             (4 + 0)

#define AM_DEVICES_COOPER_PLF_REGISTER_READ_OPCODE         0xFC39
#define AM_DEVICES_COOPER_PLF_REGISTER_READ_LENGTH         (4 + 4)

#define AM_DEVICES_COOPER_PLF_REGISTER_WRITE_OPCODE        0xFC3A
#define AM_DEVICES_COOPER_PLF_REGISTER_WRITE_LENGTH        (4 + 8)

#define AM_DEVICES_COOPER_UPDATE_FW_OPCODE                 0xFC75
#define AM_DEVICES_COOPER_UPDATE_FW_LENGTH                 (4 + 0)

#define AM_DEVICES_COOPER_UPDATE_NVDS_OPCODE               0xFC77
#define AM_DEVICES_COOPER_UPDATE_NVDS_PARA(...)            {0x4e, 0x56, 0x44, 0x53, __VA_ARGS__}
#define AM_DEVICES_COOPER_UPDATE_NVDS_LENGTH               (4 + 240)

#define AM_DEVICES_COOPER_PLF_RESET_OPCODE                 0xFC11
#define AM_DEVICES_COOPER_PLF_RESET_LENGTH                 (4 + 1)

#define AM_DEVICES_COOPER_SET_BD_ADDR_CMD(...)         {0x01, 0x74, 0xFC, 0x06, __VA_ARGS__}
#define AM_DEVICES_COOPER_SET_TX_POWER_CMD(...)        {0x01, 0x70, 0xFC, 0x01, __VA_ARGS__}
#define AM_DEVICES_COOPER_GET_DEVICE_ID_CMD()          {0x01, 0x76, 0xFC, 0x00}
#define AM_DEVICES_COOPER_PLF_REGISTER_READ_CMD(...)   {0x01, 0x01, 0xFC, 0x06, __VA_ARGS__}
#define AM_DEVICES_COOPER_PLF_REGISTER_WRITE_CMD(...)  {0x01, 0x02, 0xFC, 0x0A, __VA_ARGS__}
#define AM_DEVICES_COOPER_UPDATE_FW_CMD()              {0x01, 0x75, 0xFC, 0x00}
#define AM_DEVICES_COOPER_UPDATE_NVDS_CMD(...)         {0x01, 0x77, 0xFC, 0xF0, 0x4e, 0x56, 0x44, 0x53, __VA_ARGS__}
#define AM_DEVICES_COOPER_PLF_RESET_CMD(...)           {0x01, 0x11, 0xFC, 0x01, __VA_ARGS__}

//*****************************************************************************
//
// SBL Defines
//
//
//*****************************************************************************


#define USE_SPI_PIN                     19

//
// Slave interrupt pin is connected here
//
#define BOOTLOADER_HANDSHAKE_PIN        42

//
// This pin is connected to RESET pin of slave
//
#define DRIVE_SLAVE_RESET_PIN           17

//
// This pin is connected to the 'Override' pin of slave
//
#define DRIVE_SLAVE_OVERRIDE_PIN        4

#define AM_DEVICES_COOPER_SBL_UPDATE_STATE_INIT                     0x00
#define AM_DEVICES_COOPER_SBL_UPDATE_STATE_HELLO                    0x01
#define AM_DEVICES_COOPER_SBL_UPDATE_STATE_UPDATE                   0x02
#define AM_DEVICES_COOPER_SBL_UPDATE_STATE_DATA                     0x03
#define AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK                 0x04

#define AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW                  0x00
#define AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0              0x01
#define AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1              0x02
#define AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_NONE                0x03

#define AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE               0x200
#define AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE                 0x40

#define AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT                    0x03

#define AM_DEVICES_COOPER_SBL_ACK_RESP_SUCCESS                      0x00
#define AM_DEVICES_COOPER_SBL_ACK_RESP_FAIL                         0x01
#define AM_DEVICES_COOPER_SBL_ACK_RESP_BAD_HDR                      0x02
#define AM_DEVICES_COOPER_SBL_ACK_RESP_BAD_CRC                      0x03
#define AM_DEVICES_COOPER_SBL_ACK_RESP_VERSION_INVALID              0x04
#define AM_DEVICES_COOPER_SBL_ACK_RESP_MSG_TOO_BIG                  0x05
#define AM_DEVICES_COOPER_SBL_ACK_RESP_UNKNOWN_MSG                  0x06
#define AM_DEVICES_COOPER_SBL_ACK_RESP_INVALID_ADDRESS              0x07
#define AM_DEVICES_COOPER_SBL_ACK_RESP_INVALID_OPERATION            0x08
#define AM_DEVICES_COOPER_SBL_ACK_RESP_INVALID_PARAM                0x09
#define AM_DEVICES_COOPER_SBL_ACK_RESP_INVALID_DATA_LEN             0x0A
#define AM_DEVICES_COOPER_SBL_ACK_RESP_SEQ                          0x0B
#define AM_DEVICES_COOPER_SBL_ACK_RESP_TOO_BIG_DATA                 0x0C
#define AM_DEVICES_COOPER_SBL_ACK_RESP_BAD_IMAGE                    0x0D
#define AM_DEVICES_COOPER_SBL_ACK_RESP_FLASH_WRITE_FAILED           0x0E
#define AM_DEVICES_COOPER_SBL_ACK_RESP_INVALID_DEVICE_ID            0x0F
#define AM_DEVICES_COOPER_SBL_ACK_RESP_BAD_KEY                      0x10

#define AM_DEVICES_COOPER_SBL_STAT_RESP_SUCCESS                     0x00
#define AM_DEVICES_COOPER_SBL_STAT_RESP_FAIL                        0x01
#define AM_DEVICES_COOPER_SBL_STAT_RESP_BAD_HDR                     0x02
#define AM_DEVICES_COOPER_SBL_STAT_RESP_BAD_CRC                     0x03
#define AM_DEVICES_COOPER_SBL_STAT_RESP_STAT_BAD_SBL                0x04
#define AM_DEVICES_COOPER_SBL_STAT_RESP_BAD_IMAGE                   0x05
#define AM_DEVICES_COOPER_SBL_STAT_RESP_UNKNOWN_MSG                 0x06
#define AM_DEVICES_COOPER_SBL_STAT_RESP_FW_UPDATE_REQ               0x07
#define AM_DEVICES_COOPER_SBL_STAT_RESP_INFO0_UPDATE_REQ            0x08
#define AM_DEVICES_COOPER_SBL_STAT_RESP_INFO1_UPDATE_REQ            0x09

#define AM_DEVICES_COOPER_SBL_STATUS_INIT                           0x00000000
#define AM_DEVICES_COOPER_SBL_STATUS_OK                             0xA5A5A5A5
#define AM_DEVICES_COOPER_SBL_STATUS_FAIL                           0xA1A1A1A1
#define AM_DEVICES_COOPER_SBL_STATUS_CRC_FAIL                       0xA2A2A2A2
#define AM_DEVICES_COOPER_SBL_STATUS_UPDATE_FW                      0x4598F231
#define AM_DEVICES_COOPER_SBL_STATUS_UPDATE_INFO_0                  0x8730DA5B
#define AM_DEVICES_COOPER_SBL_STATUS_UPDATE_INFO_1                  0x09FA3725
#define AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS                    0xA78BD32C
#define AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL              0xA3A3A3A3

// Signatures for the image downloads
#define AM_DEVICES_COOPER_SBL_BLE_FW_AVAILABLE_SIGN                 0x6A, 0x87, 0x93, 0xC5
#define AM_DEVICES_COOPER_SBL_INFO_0_PATCH_AVAILABLE_SIGN           0xC9, 0x18, 0x5D, 0xB3
#define AM_DEVICES_COOPER_SBL_INFO_1_PATCH_AVAILABLE_SIGN           0x0D, 0x5A, 0xB7, 0x38

#define AM_DEVICES_COOPER_SBL_FW_VERSION                            0x0000010D
#define AM_DEVICES_COOPER_SBL_DEFAULT_FW_VERSION                    0xFFFFFFFF

#define AM_DEVICES_COOPER_SBL_STAT_VER_ROLL_BACK_EN                 0xFFFFFFFF
#define AM_DEVICES_COOPER_SBL_STAT_VER_ROLL_BACK_DBL                0xFFFFFF00

#define AM_DEVICES_COOPER_SBL_FW_IMAGE_ADDRESS                      0x10030000
#define AM_DEVICES_COOPER_SBL_FW_IMAGE_SIZE                         0x22890
#define AM_DEVICES_COOPER_SBL_FW_IMAGE_SIZE_MAX                     0x30000

#define AM_DEVICES_COOPER_SBL_INFO0_PATCH_ADDRESS                   AM_DEVICES_COOPER_SBL_FW_IMAGE_ADDRESS +  AM_DEVICES_COOPER_SBL_FW_IMAGE_SIZE_MAX
#define AM_DEVICES_COOPER_SBL_INFO0_PATCH_SIZE                      0x180
#define AM_DEVICES_COOPER_SBL_INFO0_PATCH_SIZE_MAX                  0x200

#define AM_DEVICES_COOPER_SBL_INFO1_PATCH_ADDRESS                   AM_DEVICES_COOPER_SBL_INFO0_PATCH_ADDRESS +  AM_DEVICES_COOPER_SBL_INFO0_PATCH_SIZE_MAX
#define AM_DEVICES_COOPER_SBL_INFO1_PATCH_SIZE                      0x80
#define AM_DEVICES_COOPER_SBL_INFO1_PATCH_SIZE_MAX                  0x100

typedef struct
{
    uint8_t*    pImageAddress;
    uint32_t    imageSize;
    uint32_t    imageType;
    uint32_t    version;
} am_devices_cooper_sbl_update_data_t;


typedef struct
{
    uint32_t    ui32SblUpdateState;
    uint8_t*    pImageBuf;
    uint32_t    ui32ImageSize;
    uint8_t*    pDataBuf;
    uint32_t    ui32DataSize;
    uint32_t    ui32PacketNumber;
    uint32_t    ui32TotalPackets;
    uint32_t    ui32ImageType;
    uint32_t    ui32ErrorCounter;
    void*       pHandle; // cooper_device handle
    uint32_t*   pWorkBuf;

    uint32_t    ui32CooperFWImageVersion;
    uint32_t    ui32CooperSblStatus;
    uint32_t    ui32CooperVerRollBackConfig; //Version 2
} am_devices_cooper_sbl_update_state_t;


typedef struct
{
    uint32_t                     crc32;   // First word
    uint16_t                     msgType; // am_secboot_wired_msgtype_e
    uint16_t                     length;
} am_secboot_wired_msghdr_t;

typedef struct
{
    uint32_t                      length  : 16;
    uint32_t                      resv    : 14;
    uint32_t                      bEnd    : 1;
    uint32_t                      bStart  : 1;
} am_secboot_ios_pkthdr_t;


//Message types
typedef enum
{
    AM_SBL_HOST_MSG_HELLO = 0,
    AM_SBL_HOST_MSG_STATUS,
    AM_SBL_HOST_MSG_UPDATE_STATUS,
    AM_SBL_HOST_MSG_UPDATE,
    AM_SBL_HOST_MSG_FW_CONTINUE,
    AM_SBL_HOST_MSG_NACK,
    AM_SBL_HOST_MSG_RESET,
    AM_SBL_HOST_MSG_ACK,
    AM_SBL_HOST_MSG_DATA
} AM_SBL_HOST_MSG_E;

///////////////////SBL Messages To and From Host////////////////////////

// Message header
typedef struct
{
    uint32_t    msgCrc;
    uint16_t    msgType;
    uint16_t    msgLength;
} am_sbl_host_msg_hdr_t;

// Hello Message
typedef struct
{
    am_sbl_host_msg_hdr_t    msgHdr;
} am_sbl_host_msg_hello_t;

// Status message
typedef struct
{
    am_sbl_host_msg_hdr_t   msgHdr;
    uint32_t                versionNumber;
    uint32_t                maxImageSize;
    uint32_t                bootStatus;
    uint32_t                verRollBackStatus; //Version 2
    uint32_t                copperChpIdWord0;  //Version 2
    uint32_t                copperChpIdWord1;  //Version 2
} am_sbl_host_msg_status_t;

//Update Message
typedef struct
{
    am_sbl_host_msg_hdr_t   msgHdr;
    uint32_t                imageSize;
    uint32_t                maxPacketSize;
    uint32_t                NumPackets;
    uint32_t                versionNumber;
} am_sbl_host_msg_update_t;

//Data Message
typedef struct
{
    am_sbl_host_msg_hdr_t   msgHdr;
    uint32_t                packetNumber;
    uint8_t                 data[];
} am_sbl_host_msg_data_t;

//FW continue Message
typedef struct
{
    am_sbl_host_msg_hdr_t   msgHdr;
    uint32_t                reserved;
} am_sbl_host_msg_fw_continue_t;

//Reset Message
typedef struct
{
    am_sbl_host_msg_hdr_t   msgHdr;
    uint32_t                reserved;
} am_sbl_host_msg_reset_t;

//Ack /Nack Message
typedef struct
{
    am_sbl_host_msg_hdr_t   msgHdr;
    uint32_t                srcMsgType;
    uint32_t                status;
    uint32_t                nextPacketNum;  // only valid for data messages ack
    uint32_t                reserved[3];    //Version 2
} am_sbl_host_msg_ack_nack_t;


#define   AM_DEVICES_COOPER_SBL_MAX_INFO_0_PATCH_VALUES         64
#define   AM_DEVICES_COOPER_SBL_BIT_MASK_PER_WORD               16
#define   AM_DEVICES_COOPER_SBL_INFO_0_REPLACE_TRIM        0x00000003

// INFO 0 patch for Rev A only
typedef struct
{
    uint32_t     magicNumNSize;             // 0x0A500180
    uint32_t    rsaSize;            // 0x00000000
    uint32_t    loadHdrReserved[14];            // All Zeros
    uint32_t    cmacHash[4];            // All Zeros
    uint32_t    authKeyNHashDataSize;            // All Zeros
    uint32_t    xBlock;                // 0x00000000
    uint32_t    keyDervData[4];            // All Zeros

    uint32_t    bitMaskWord[4];
    uint32_t    reserved_0;
    uint32_t    trimDataWords[AM_DEVICES_COOPER_SBL_MAX_INFO_0_PATCH_VALUES];
    uint32_t    reserved_1;

} am_sbl_info0_patch_blob_t;

// INFO 0 patch data
typedef struct
{
    uint32_t    wordOffset;
    uint32_t    value;
} am_sbl_info0_patch_data_t;

uint32_t am_devices_cooper_update_image(void);
uint32_t am_devices_cooper_image_update_init(void* pHandle, uint32_t* pWorkbuf);
bool am_devices_cooper_get_FwImage(am_devices_cooper_sbl_update_data_t *pFwImage );
bool am_devices_cooper_get_info1_patch(am_devices_cooper_sbl_update_data_t *pInfo1Image);
bool am_devices_cooper_get_info0_patch(am_devices_cooper_sbl_update_data_t *pInfo0Image);

//*****************************************************************************
//
// HCI packet types.
//
//*****************************************************************************
#define AM_DEVICES_COOPER_RAW                      0x0
#define AM_DEVICES_COOPER_CMD                      0x1
#define AM_DEVICES_COOPER_ACL                      0x2
#define AM_DEVICES_COOPER_EVT                      0x4

typedef struct
{
    uint32_t* pNBTxnBuf;
    uint32_t ui32NBTxnBufLength;
} am_devices_cooper_config_t;

typedef void (*am_devices_cooper_callback_t)(void* pCallbackCtxt);

typedef struct
{
    uint32_t                     ui32Module;
    uint32_t                     ui32CS;
    am_devices_cooper_callback_t pfnCallback;
    void*                         pCallbackCtxt;
    void*                         pBleHandle;
    bool                         bOccupied;
    bool                         bBusy;
    bool                         bNeedCallback;
    volatile bool                bDMAComplete;
    bool                         bWakingUp;
} am_devices_cooper_t;

#define AM_DEVICES_COOPER_MAX_DEVICE_NUM 1

extern void am_devices_cooper_pins_enable(void);
extern void am_devices_cooper_pins_disable(void);
extern uint32_t am_devices_cooper_init(uint32_t ui32Module, am_devices_cooper_config_t* pDevConfig, void** ppHandle, void** ppBleHandle);
extern uint32_t am_devices_cooper_term(void* pHandle);
extern void am_devices_cooper_reset(void);
extern uint32_t am_devices_cooper_awake(void* pHandle);
extern uint32_t am_devices_cooper_sleep(void* pHandle);
extern uint32_t am_devices_cooper_blocking_read(void* pHandle, uint32_t* pui32Data, uint32_t* pui32BytesReceived);
extern uint32_t am_devices_cooper_blocking_write(void* pHandle, uint8_t ui8Type, uint32_t* pui32Data, uint32_t ui32NumBytes, bool bWaitReady);
extern uint32_t am_devices_cooper_irq_read(void);
extern uint32_t am_devices_cooper_clkreq_read(void* pHandle);
extern uint32_t am_devices_cooper_crystal_trim_set(void *pHandle, uint32_t ui32TrimValue);
extern uint32_t am_devices_cooper_reset_with_sbl_check(void* pHandle, am_devices_cooper_config_t* pDevConfig);
extern uint32_t am_devices_cooper_disable_rollback(void* pHandle, am_devices_cooper_config_t* pDevConfig);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_COOPER_H
