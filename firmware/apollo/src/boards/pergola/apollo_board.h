/**
 * Apollo board definitions for Pergola hardware.
 *
 * This file is part of LUNA.
 *
 * Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
 * Copyright (c) 2020 Konrad Beckmann
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __APOLLO_BOARD_H__
#define __APOLLO_BOARD_H__

#include "fsl_gpio.h"
#include <inttypes.h>
#include <stdbool.h>

/**
 * Simple-but-hacky macros that allow us to treat a GPIO pin as a single object.
 * Takes on some nastiness here to hide some of the vendor-library nastiness.
 */
#define _PERGOLA_GPIO(port, pin)  ((uint16_t)port << 8) | pin
#define _PERGOLA_PORT(gpio)       (_pergola_gpio_port_map[gpio >> 8])
#define _PERGOLA_PIN(gpio)        (gpio & 0xFF)

// Create a quick alias for a GPIO type.
typedef uint16_t gpio_t;

extern GPIO_Type * _pergola_gpio_port_map[];

#define _GPIO1 0
#define _GPIO2 1
#define _GPIO5 2

/**
 * GPIO pins for each of the microcontroller LEDs.
 */
typedef enum {
	LED_G = _PERGOLA_GPIO(_GPIO1, 1),
	LED_R = _PERGOLA_GPIO(_GPIO1, 2),
	LED_B = _PERGOLA_GPIO(_GPIO1, 3),
} led_t;


/**
 * Debug SPI pin locations.
 */
enum {
	PIN_SCK      = _PERGOLA_GPIO(_GPIO1, 4), // FIXME
	PIN_SDI      = _PERGOLA_GPIO(_GPIO1, 4), // FIXME
	PIN_SDO      = _PERGOLA_GPIO(_GPIO1, 4), // FIXME
	PIN_FPGA_CS  = _PERGOLA_GPIO(_GPIO1, 4), // FIXME

	PIN_PROGRAMN = _PERGOLA_GPIO(_GPIO1, 21),
	PIN_DONE     = _PERGOLA_GPIO(_GPIO1, 15),
};


/**
 * GPIO pin numbers for each of the JTAG pins.
 */
enum {
	TDO_GPIO = _PERGOLA_GPIO(_GPIO1, 17),
	TDI_GPIO = _PERGOLA_GPIO(_GPIO1, 18),
	TMS_GPIO = _PERGOLA_GPIO(_GPIO1, 19),
	TCK_GPIO = _PERGOLA_GPIO(_GPIO1, 20),
};



/**
 * GPIO abstractions; hide vendor code.
 */
enum {
	GPIO_DIRECTION_IN  = 0,
	GPIO_DIRECTION_OUT = 1,

	GPIO_PULL_OFF = 0,
	GPIO_PIN_FUNCTION_OFF = 0,
};


static inline void gpio_set_pin_level(uint16_t pin, bool state)
{
	GPIO_PinWrite(_PERGOLA_PORT(pin), _PERGOLA_PIN(pin), state);
}


static inline bool gpio_get_pin_level(uint16_t pin)
{
	return GPIO_PinRead(_PERGOLA_PORT(pin), _PERGOLA_PIN(pin));
}


static inline void gpio_toggle_pin_level(uint16_t pin)
{
	gpio_set_pin_level(pin, !gpio_get_pin_level(pin));
}


static inline void gpio_set_pin_direction(uint16_t pin, uint8_t direction)
{
	gpio_pin_config_t config = {
		direction ? kGPIO_DigitalOutput : kGPIO_DigitalInput,
		0,
		kGPIO_NoIntmode
	};
	GPIO_PinInit(_PERGOLA_PORT(pin), _PERGOLA_PIN(pin), &config);
}


static inline void gpio_set_pin_pull_mode(uint16_t pin, uint8_t any)
{
	// TODO: NOP for now
}


static inline void gpio_set_pin_function(uint16_t pin, uint8_t any)
{
	// TODO: NOP for now
}




#endif
