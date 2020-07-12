/*
 * SPI driver code.
 *
 * This file is part of LUNA.
 *
 * Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "spi.h"
#include "led.h"

#include <bsp/board.h>
#include <apollo_board.h>
#include <platform_jtag.h>

/**
 * Configures the relevant SPI target's pins to be used for SPI.
 */
void spi_configure_pinmux(spi_target_t target)
{
	if (target == SPI_FPGA_JTAG) {
		gpio_set_pin_direction(TDO_GPIO, GPIO_DIRECTION_IN);
		gpio_set_pin_direction(TDI_GPIO, GPIO_DIRECTION_OUT);
		gpio_set_pin_direction(TMS_GPIO, GPIO_DIRECTION_OUT);
		gpio_set_pin_direction(TCK_GPIO, GPIO_DIRECTION_OUT);
	}
}


/**
 * Returns the relevant SPI target's pins to being used for GPIO.
 */
void spi_release_pinmux(spi_target_t target)
{
	// nop
}


/**
 * Configures the provided target to be used as an SPI port via the SERCOM.
 */
void spi_init(spi_target_t target, bool lsb_first, bool configure_pinmux, uint8_t baud_divider,
	 uint8_t clock_polarity, uint8_t clock_phase)
{
	// nop
}


static inline uint8_t jtag_pulse_clock_and_read_tdo(void)
{
	uint8_t ret;

	gpio_set_pin_level(TCK_GPIO, false);
	__NOP();
	ret = jtag_read_tdo();
	gpio_set_pin_level(TCK_GPIO, true);

	return ret;
}

/**
 * Synchronously send a single byte on the given SPI bus.
 * Does not manage the SSEL line.
 */
uint8_t spi_send_byte(spi_target_t port, uint8_t data)
{
	uint8_t tdo_byte = 0;
	for (int j = 0; j < 8; ++j) {
		if (data & 1) {
			jtag_set_tdi();
		} else {
			jtag_clear_tdi();
		}
		data >>= 1;
		bool tdo = jtag_pulse_clock_and_read_tdo();
		tdo_byte |= tdo << j;
	}

	return tdo_byte;
}


/**
 * Sends a block of data over the SPI bus.
 *
 * @param port The port on which to perform the SPI transaction.
 * @param data_to_send The data to be transferred over the SPI bus.
 * @param data_received Any data received during the SPI transaction.
 * @param length The total length of the data to be exchanged, in bytes.
 */
void spi_send(spi_target_t port, void *data_to_send, void *data_received, size_t length)
{
	uint8_t *to_send  = data_to_send;
	uint8_t *received = data_received;

	for (unsigned i = 0; i < length; ++i) {
		received[i] = spi_send_byte(port, to_send[i]);
	}
}
