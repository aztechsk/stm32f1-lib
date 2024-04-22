/*
 * sigidle.h
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

#ifndef SIGIDLE_H
#define SIGIDLE_H

#ifndef SIGIDLE
 #define SIGIDLE 0
#endif

#if SIGIDLE == 1

#include "gpio.h"
#include "tim.h"

typedef struct sigidle_dsc *sigidle;

struct sigidle_dsc {
        enum gpio_pin pin; // <SetIt>
	GPIO_TypeDef *port; // <SetIt>
        boolean_t idle_lev; // <SetIt>
        TIM_TypeDef *tim_mmio; // <SetIt>
        int tm_us; // <SetIt>
        TaskHandle_t c_tsk;
        timer tim;
        struct sigidle_dsc *next;
};

/**
 * init_sigidle
 *
 * Configures SIGIDLE instance.
 *
 * @dev: SIGIDLE instance.
 */
void init_sigidle(sigidle dev);

/**
 * sigidle_wait
 *
 * Waits for idle state on input signal for defined minimum time.
 *
 * @dev: SIGIDLE instance.
 */
void sigidle_wait(sigidle dev);

#endif

#endif
