/*
 * gpio.h
 *
 * Copyright (c) 2022 Jan Rusnak <jan@rusnak.sk>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef GPIO_H
#define GPIO_H

/**
 * init_gpio_port
 *
 * Enables Gpio instance clock.
 *
 * @port: Gpio peripheral instance.
 */
void init_gpio_port(GPIO_TypeDef *port);

/**
 * disable_gpio_port
 *
 * Disables Gpio instance clock.
 *
 * @port: Gpio peripheral instance.
 */
void disable_gpio_port(GPIO_TypeDef *port);

enum gpio_pin {
	GPIO_PIN0 =  0x0001,
        GPIO_PIN1 =  0x0002,
        GPIO_PIN2 =  0x0004,
        GPIO_PIN3 =  0x0008,
        GPIO_PIN4 =  0x0010,
        GPIO_PIN5 =  0x0020,
        GPIO_PIN6 =  0x0040,
        GPIO_PIN7 =  0x0080,
        GPIO_PIN8 =  0x0100,
        GPIO_PIN9 =  0x0200,
        GPIO_PIN10 = 0x0400,
        GPIO_PIN11 = 0x0800,
        GPIO_PIN12 = 0x1000,
        GPIO_PIN13 = 0x2000,
        GPIO_PIN14 = 0x4000,
        GPIO_PIN15 = 0x8000
};

enum gpio_pin_func {
        GPIO_FUNC_INPUT,
	GPIO_FUNC_ANALOG,
	GPIO_FUNC_OUTPUT,
	GPIO_FUNC_OUTPUT_DRAIN,
	GPIO_FUNC_PERIPH,
        GPIO_FUNC_PERIPH_DRAIN
};

enum gpio_pin_feat {
	GPIO_FEAT_PULL_UP,
        GPIO_FEAT_PULL_DOWN,
        GPIO_FEAT_LEV_0,
	GPIO_FEAT_LEV_1,
	GPIO_FEAT_SPEED_LOW,
        GPIO_FEAT_SPEED_NORM,
	GPIO_FEAT_SPEED_HIGH,
        GPIO_FEAT_END
};

/**
 * conf_gpio
 *
 * Configures Gpio pin.
 *
 * GPIO_FUNC_INPUT
 *  = Input floating.
 *
 * GPIO_FUNC_INPUT + GPIO_FEAT_PULL_UP
 *  = Input pull-up.
 *
 * GPIO_FUNC_INPUT + GPIO_FEAT_PULL_DOWN
 *  = Input pull-down.
 *
 * GPIO_FUNC_ANALOG
 *  = Analog pin.
 *
 * GPIO_FUNC_OUTPUT + GPIO_FEAT_SPEED_* + (GPIO_FEAT_LEV_0 | GPIO_FEAT_LEV_1)
 *  = Push-pull output.
 *
 * GPIO_FUNC_OUTPUT_DRAIN + GPIO_FEAT_SPEED_* + (GPIO_FEAT_LEV_0 | GPIO_FEAT_LEV_1)
 *  = Open-drain output.
 *
 * GPIO_FUNC_PERIPH + GPIO_FEAT_SPEED_*
 *  = Peripheral push-pull output.
 *
 * GPIO_FUNC_PERIPH_DRAIN + GPIO_FEAT_SPEED_*
 *  = Peripheral open-drain output.
 *
 * @pin: Pin number (enum gpio_pin).
 * @port: Gpio peripheral instance.
 * @func: Pin function definition (enum gpio_pin_func).
 * @va: List of additional pin features terminated with GPIO_FEAT_END (enum gpio_pin_feat).
 */
void conf_gpio(enum gpio_pin pin, GPIO_TypeDef *port, enum gpio_pin_func func, ...);

/**
 * get_gpio_lev
 *
 * Gets logical level of input pin.
 *
 * @pin: Pin number (enum gpio_pin).
 * @port: Gpio peripheral instance.
 *
 * Returns: HIGH; LOW.
 */
inline boolean_t get_gpio_lev(enum gpio_pin pin, GPIO_TypeDef *port)
{
	return ((port->IDR & pin) ? HIGH : LOW);
}

/**
 * set_gpio_lev
 *
 * Sets logical level of output pin.
 *
 * @pin: Pin number (enum gpio_pin).
 * @port: Gpio peripheral instance.
 * @lev: Logical level.
 */
inline void set_gpio_lev(enum gpio_pin pin, GPIO_TypeDef *port, boolean_t lev)
{
	if (lev) {
		port->BSRR = pin;
	} else {
		port->BRR = pin;
	}
}

/**
 * get_gpio_out
 *
 * Gets logical level setting of output pin.
 *
 * @pin: Pin number (enum gpio_pin).
 * @port: Gpio peripheral instance.
 *
 * Returns: HIGH; LOW.
 */
inline boolean_t get_gpio_out(enum gpio_pin pin, GPIO_TypeDef *port)
{
	return ((port->ODR & pin) ? HIGH : LOW);
}

#endif
