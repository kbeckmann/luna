/**
 * Code for basic FPGA interfacing.
 *
 * This file is part of LUNA.
 *
 * Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bsp/board.h>
#include <apollo_board.h>

/**
 * Sets up the I/O pins needed to configure the FPGA.
 */
void fpga_io_init(void)
{
	gpio_set_pin_direction(PIN_PROGRAMN, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(PIN_DONE,     GPIO_DIRECTION_IN);
}


/**
 * Requests that the FPGA clear its configuration and try to reconfigure.
 */
void trigger_fpga_reconfiguration(void)
{
	gpio_set_pin_direction(PIN_PROGRAMN, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PROGRAMN, false);

	board_delay(1);

	gpio_set_pin_level(PIN_PROGRAMN, true);
}
