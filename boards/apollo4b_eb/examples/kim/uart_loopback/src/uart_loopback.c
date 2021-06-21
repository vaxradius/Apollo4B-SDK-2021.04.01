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

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "am_util.h"
#include "am_bsp.h"

//*****************************************************************************
//
// Macro Definition.
//
//*****************************************************************************
#define UART_HCI_BRIDGE                 (3)
#define MAX_UART_PACKET_SIZE            (2048)
#define UART_RX_TIMEOUT_MS              (1)
#define MAX_READ_BYTES                  (23)

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
void *g_pvUART;
uint8_t g_pui8UARTTXBuffer[MAX_UART_PACKET_SIZE];

#define am_uart_buffer(A)                                                   \
union                                                                   \
  {                                                                       \
    uint32_t words[(A + 3) >> 2];                                       \
      uint8_t bytes[A];                                                   \
  }

am_uart_buffer(1024) g_psWriteData;
//am_uart_buffer(1024) g_psReadData;

//*****************************************************************************
//
// function declaration.
//
//*****************************************************************************
volatile uint32_t g_ui32SerialRxIndex = 0;
void serial_data_read(uint8_t* pui8Data, uint32_t* ui32Length);

//*****************************************************************************
//
// Interrupt handler for the UART.
//
//*****************************************************************************
#if UART_HCI_BRIDGE == 0
void am_uart_isr(void)
#else
void am_uart3_isr(void)
#endif //UART_HCI_BRIDGE == 0
{
    uint32_t ui32Status;
    uint8_t * pData = (uint8_t *) &(g_psWriteData.bytes[g_ui32SerialRxIndex]);

    //
    // Read the masked interrupt status from the UART.
    //
    am_hal_uart_interrupt_status_get(g_pvUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(g_pvUART, ui32Status);
    am_hal_uart_interrupt_service(g_pvUART, ui32Status);
    //
    // If there's an RX interrupt, handle it in a way that preserves the
    // timeout interrupt on gaps between packets.
    //
    if (ui32Status & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_RX))
    {
        uint32_t ui32BytesRead;
        serial_data_read(pData, &ui32BytesRead);
        g_ui32SerialRxIndex += ui32BytesRead;
    }
}

am_hal_gpio_pincfg_t g_AM_GPIO13_COM_UART2_TX =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_13_UART3TX,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

am_hal_gpio_pincfg_t g_AM_GPIO11_COM_UART2_RX =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_11_UART3RX,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//*****************************************************************************
//
// Initialize the UART.
//
//*****************************************************************************
void serial_interface_init(void)
{
    //
    // Start the UART.
    //
    am_hal_uart_config_t sUartConfig =
    {
        //
        // Standard UART settings: 115200-8-N-1
        //
        .ui32BaudRate    = 115200,
        //
        // Set TX and RX FIFOs to interrupt at three-quarters full.
        //
        .eDataBits    = AM_HAL_UART_DATA_BITS_8,
        .eParity      = AM_HAL_UART_PARITY_NONE,
        .eStopBits    = AM_HAL_UART_ONE_STOP_BIT,
        .eFlowControl = AM_HAL_UART_FLOW_CTRL_NONE,
        .eTXFifoLevel = AM_HAL_UART_FIFO_LEVEL_28,
        .eRXFifoLevel = AM_HAL_UART_FIFO_LEVEL_28,
    };

    am_hal_uart_initialize(UART_HCI_BRIDGE, &g_pvUART);
    am_hal_uart_power_control(g_pvUART, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_uart_configure(g_pvUART, &sUartConfig);
    am_hal_uart_buffer_configure(g_pvUART, g_pui8UARTTXBuffer, sizeof(g_pui8UARTTXBuffer), NULL, 0);
    am_hal_gpio_pinconfig(13, g_AM_GPIO13_COM_UART2_TX);
    am_hal_gpio_pinconfig(11, g_AM_GPIO11_COM_UART2_RX);
    //
    // Make sure to enable the interrupts for RX, since the HAL doesn't already
    // know we intend to use them.
    //
    NVIC_SetPriority((IRQn_Type)(UART0_IRQn + UART_HCI_BRIDGE), AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + UART_HCI_BRIDGE));
    am_hal_uart_interrupt_enable(g_pvUART, (AM_HAL_UART_INT_RX |
                                 AM_HAL_UART_INT_RX_TMOUT));
}

//*****************************************************************************
//
// Deinitialize the UART.
//
//*****************************************************************************
void serial_interface_deinit(void)
{
    am_hal_gpio_pincfg_t sPinCfg;
    sPinCfg.GP.cfg_b.uFuncSel  = 3; //GPIO
    sPinCfg.GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_DISABLE;
    sPinCfg.GP.cfg_b.eGPInput  = AM_HAL_GPIO_PIN_INPUT_ENABLE;
    sPinCfg.GP.cfg_b.ePullup   = AM_HAL_GPIO_PIN_PULLUP_100K;
    am_hal_gpio_pinconfig(13, sPinCfg);
    am_hal_gpio_pinconfig(11, sPinCfg);

    am_hal_uart_deinitialize(g_pvUART);
}

//*****************************************************************************
//
// Read UART data.
//
//*****************************************************************************
void serial_data_read(uint8_t* pui8Data, uint32_t* ui32Length)
{
    am_hal_uart_transfer_t sRead =
    {
        .eType = AM_HAL_UART_BLOCKING_READ,
        .ui32TimeoutMs = UART_RX_TIMEOUT_MS,
        .pui8Data = pui8Data,
        .ui32NumBytes = MAX_READ_BYTES,
        .pui32BytesTransferred = ui32Length,
    };

    am_hal_uart_transfer(g_pvUART, &sRead);
}

//*****************************************************************************
//
// Write UART data.
//
//*****************************************************************************
void serial_data_write(uint8_t* pui8Data, uint32_t ui32Length)
{
    uint32_t ui32BytesTransferred = 0;
    am_hal_uart_transfer_t sWrite =
    {
        .eType = AM_HAL_UART_NONBLOCKING_WRITE,
        .pui8Data = pui8Data,
        .ui32NumBytes = ui32Length,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = &ui32BytesTransferred,
    };

    am_hal_uart_transfer(g_pvUART, &sWrite);
}

//*****************************************************************************
//
// Enable UART IRQ.
//
//*****************************************************************************
void serial_irq_enable(void)
{
    NVIC_SetPriority((IRQn_Type)(UART0_IRQn + UART_HCI_BRIDGE), AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + UART_HCI_BRIDGE));
}

//*****************************************************************************
//
// Disable UART IRQ.
//
//*****************************************************************************
void serial_irq_disable(void)
{
    NVIC_DisableIRQ((IRQn_Type)(UART0_IRQn + UART_HCI_BRIDGE));
}


//*****************************************************************************
//
// Main
//
//*****************************************************************************
int main(void)
{
	uint32_t ui32Critical=0; 
	serial_interface_init();

	serial_irq_enable();

	am_hal_interrupt_master_enable();

	//
    // Initialize the printf interface for UART output.
    //
    am_bsp_uart_printf_enable();

    am_util_stdio_printf("Uart3 loopback Example\n");

	while (1)
    {

		if(g_ui32SerialRxIndex)
		{
			uint8_t * pData = (uint8_t *) &(g_psWriteData.bytes[0]);
			ui32Critical = am_hal_interrupt_master_disable(); //Start a critical section.
			serial_data_write(pData, g_ui32SerialRxIndex);
			g_ui32SerialRxIndex = 0;
			am_hal_interrupt_master_set(ui32Critical); //Exit the critical section.
		}
		//
        // Go to Deep Sleep and stay there.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
    }
}

