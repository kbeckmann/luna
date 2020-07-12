/*
 * LED control abstraciton code.
 *
 * This file is part of LUNA.
 *
 * Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <tusb.h>
#include <bsp/board.h>
#include <apollo_board.h>


#include "led.h"

/** Store the current LED blink pattern. */
static blink_pattern_t blink_pattern = BLINK_IDLE;

#define UART_LED_TMO 250

static uint32_t uart_rx_tmr;
static uint32_t uart_tx_tmr;

/**
 * Sets the active LED blink pattern.
 */
void led_set_blink_pattern(blink_pattern_t pattern)
{
	blink_pattern = pattern;
	leds_off();
}


/**
 * Sets up each of the LEDs for use.
 */
void led_init(void)
{
	gpio_set_pin_direction(LED_R, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(LED_G, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(LED_B, GPIO_DIRECTION_OUT);
	leds_off();
}


/**
 * Turns the provided LED on.
 */
void led_on(led_t led)
{
	gpio_set_pin_level(led, false);
}


/**
 * Turns the provided LED off.
 */
void led_off(led_t led)
{
	gpio_set_pin_level(led, true);
}


/**
 * Toggles the provided LED.
 */
void led_toggle(led_t led)
{
	gpio_toggle_pin_level(led);
}


/**
 * Sets whether a given led is on.
 */
void led_set(led_t led, bool on)
{
	gpio_set_pin_level(led, !on);
}


/**
 * Turns off all of the device's LEDs.
 */
void leds_off(void)
{
	led_off(LED_R);
	led_off(LED_G);
	led_off(LED_B);
}

void led_uart_tx(void)
{
	uart_tx_tmr = board_millis();
	leds_off();
	led_on(LED_R);
}

void led_uart_rx(void)
{
	uart_rx_tmr = board_millis();
	leds_off();
	led_on(LED_G);
}

/**
 * Task that handles blinking the heartbeat LED.
 */
void heartbeat_task(void)
{
	static uint32_t start_ms = 0;
	static uint32_t blink = 0;

	if ((board_millis() - uart_tx_tmr < UART_LED_TMO) ||
	    (board_millis() - uart_rx_tmr < UART_LED_TMO) ||
	    (board_millis() - start_ms < blink_pattern)) {
		return;
	}

	start_ms += blink_pattern;
	blink = ~blink;

	leds_off();

	switch (blink_pattern) {
	case BLINK_IDLE:
		led_set(LED_G, blink);
		break;
	case BLINK_JTAG_CONNECTED:
		led_set(LED_G, blink);
		led_set(LED_B, blink);
		break;
	case BLINK_JTAG_UPLOADING:
		led_set(LED_G, blink);
		led_set(LED_B, blink);
		break;
	case BLINK_FLASH_CONNECTED:
	default:
		led_set(LED_R, blink);
		break;
	}

}
