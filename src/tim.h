/*
 * tim.h
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

#ifndef TIM_H
#define TIM_H

#ifndef TIMER
 #define TIMER 0
#endif

#if TIMER == 1

enum timer_mode {
	TIMER_PERIODIC_ISR_MODE,
        TIMER_ONESHOT_ISR_MODE
};

typedef struct timer_dsc *timer;

struct timer_dsc {
        TIM_TypeDef *mmio; // <SetIt>
        BaseType_t (*isr_clbk)(void *); // <SetIt>
	int tm_us; // <SetIt>
	void *owner; // <SetIt> (if owner exists)
        enum timer_mode mode;
	int apb_inst;
	unsigned int apb_bmp;
	int irqn;
	unsigned int reg_cr1;
        unsigned int reg_smcr;
        unsigned int reg_psc;
        unsigned int reg_arr;
};

/**
 * init_timer
 *
 * Configures TIMER instance in requested mode.
 *
 * @dev: TIMER instance.
 * @mode: TIMER mode.
 */
void init_timer(timer dev, enum timer_mode mode);

/**
 * reconf_timer
 *
 * Reconfigures TIMER parameters according to new value of timer_dsc's
 * member tm_us.
 *
 * @dev: TIMER instance.
 */
void reconf_timer(timer dev);

/**
 * start_timer
 *
 * Starts TIMER counting.
 *
 * @dev: TIMER instance.
 */
void start_timer(timer dev);

/**
 * restart_timer
 *
 * Restarts TIMER counting.
 *
 * @dev: TIMER instance.
 */
void restart_timer(timer dev);

/**
 * stop_timer
 *
 * Stops TIMER counting.
 *
 * @dev: TIMER instance.
 */
void stop_timer(timer dev);

/**
 * clear_timer_sr
 *
 * Clears TIMER status register.
 *
 * @dev: TIMER instance.
 */
void clear_timer_sr(timer dev);

#endif

#endif
