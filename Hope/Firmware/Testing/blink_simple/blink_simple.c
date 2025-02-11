/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#define LEDPIN 8
#ifndef LED_DELAY_MS
#define LED_DELAY_MS 500
#endif

// Initialize the GPIO for the LED
void pico_led_init(void) {
#ifdef LEDPIN
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(LEDPIN);
    gpio_set_dir(LEDPIN, GPIO_OUT);
#endif
}

// Turn the LED on or off
void pico_set_led(bool led_on) {
#if defined(LEDPIN)
    // Just set the GPIO on or off
    gpio_put(LEDPIN, led_on);
#endif
}

int main() {
    pico_led_init();
    while (true) {
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
    }
}
