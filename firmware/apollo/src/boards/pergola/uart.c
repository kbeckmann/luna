/**
 * UART driver code.
 *
 * This file is part of LUNA.
 *
 * Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <apollo_board.h>
#include <bsp/board.h>
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_clock.h"
#include "fsl_lpuart.h"

#include "pergola_led.h"

bool uart_active = true;

// UART
#define UART_PORT             LPUART1
#define UART_RX_PINMUX        IOMUXC_GPIO_09_LPUART1_RXD
#define UART_TX_PINMUX        IOMUXC_GPIO_10_LPUART1_TXD

lpuart_handle_t uart_handle;

uint8_t uart_rx_buf[256];

lpuart_transfer_t uart_xfer = {
	.data = uart_rx_buf,
	.dataSize = 1,
};

/**
 * Callback issued when the UART recieves a new byte.
 */
__attribute__((weak)) void uart_byte_received_cb(uint8_t byte) {}



void uart_transfer_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
	size_t recv;

	uart_byte_received_cb(uart_rx_buf[handle->rxRingBufferHead]);

	uart_xfer.dataSize = 1;
	LPUART_TransferReceiveNonBlocking(UART_PORT, &uart_handle, &uart_xfer, &recv);

	led_uart_rx();
}


/**
 * Configures the UART we'll use for our system console.
 *
 * @param configure_pinmux If true, the pinmux will be configured for UART use during init.
 * @param baudrate The baud rate to apply, in symbols/second.
 */
void uart_init(bool configure_pinmux, unsigned long baudrate)
{
	// UART
	IOMUXC_SetPinMux( UART_TX_PINMUX, 0U);
	IOMUXC_SetPinMux( UART_RX_PINMUX, 0U);
	IOMUXC_SetPinConfig( UART_TX_PINMUX, 0x10B0u);
	IOMUXC_SetPinConfig( UART_RX_PINMUX, 0x10B0u);

	lpuart_config_t uart_config;
	LPUART_GetDefaultConfig(&uart_config);
	uart_config.baudRate_Bps = CFG_BOARD_UART_BAUDRATE;
	uart_config.enableTx = true;
	uart_config.enableRx = true;
	LPUART_Init(UART_PORT, &uart_config, (CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6U) / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U));


	LPUART_TransferCreateHandle(UART_PORT, &uart_handle, uart_transfer_callback, NULL);

	LPUART_TransferStartRingBuffer(UART_PORT, &uart_handle, uart_rx_buf, sizeof(uart_rx_buf));

	size_t recv;
	LPUART_TransferReceiveNonBlocking(UART_PORT, &uart_handle, &uart_xfer, &recv);
}

/**
 * Configures the relevant UART's target's pins to be used for UART.
 */
void uart_configure_pinmux(void)
{

}


/**
 * Releases the relevant pins from being used for UART, returning them
 * to use as GPIO.
 */
void uart_release_pinmux(void)
{

}


#include "led.h"
/**
 * Writes a byte over the Apollo console UART.
 *
 * @param byte The byte to be written.
 */
void uart_blocking_write(uint8_t byte)
{
	board_uart_write(&byte, 1);
	led_uart_tx();
}


/**
 * @return True iff the UART can accept data.
 */
bool uart_ready_for_write(void)
{
	return true;
}


/**
 * Starts a write over the Apollo console UART.

 * Does not check for readiness; it is assumed the caller knows that the
 * UART is available (e.g. by calling uart_ready_for_write).
 */
void uart_nonblocking_write(uint8_t byte)
{
	// TODO
	uart_blocking_write(byte);
}
