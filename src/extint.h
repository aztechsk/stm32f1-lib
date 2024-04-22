/*
 * extint.h
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

#ifndef EXTINT_H
#define EXTINT_H

#ifndef EXTINT
 #define EXTINT 0
#endif

#if EXTINT == 1

enum extint_line {
	EXTINT_LINE0 =  0x0001,
        EXTINT_LINE1 =  0x0002,
        EXTINT_LINE2 =  0x0004,
        EXTINT_LINE3 =  0x0008,
        EXTINT_LINE4 =  0x0010,
        EXTINT_LINE5 =  0x0020,
        EXTINT_LINE6 =  0x0040,
        EXTINT_LINE7 =  0x0080,
        EXTINT_LINE8 =  0x0100,
        EXTINT_LINE9 =  0x0200,
        EXTINT_LINE10 = 0x0400,
        EXTINT_LINE11 = 0x0800,
        EXTINT_LINE12 = 0x1000,
        EXTINT_LINE13 = 0x2000,
        EXTINT_LINE14 = 0x4000,
        EXTINT_LINE15 = 0x8000
};

/**
 * init_extint_intr
 *
 * Configures EXTINT line interrupt (NVIC).
 *
 * @line: EXTINT line (enum extint_line).
 */
void init_extint_intr(enum extint_line line);

/**
 * is_extint_intr_init
 *
 * Checks if interrupt line (NVIC) has already been initialized.
 *
 * @line: EXTINT line (enum extint_line).
 */
boolean_t is_extint_intr_init(enum extint_line line);

/**
 * map_extint_line
 *
 * Map EXTINT line to GPIO port.
 *
 * @line: EXTINT line (enum extint_line).
 * @port: GPIO port.
 *
 * AFIO must be clocked before function call.
 */
void map_extint_line(enum extint_line line, GPIO_TypeDef *port);

enum extint_line_mode {
	EXTINT_LINE_FALL_TRIG,
        EXTINT_LINE_RISE_TRIG,
	EXTINT_LINE_EDGE_TRIG
};

/**
 * set_extint_line_mode
 *
 * Sets interrupt (event) trigger mode for EXTINT line.
 *
 * @line: EXTINT line (enum extint_line).
 * @mode: Line trigger mode (enum extint_line_mode).
 */
void set_extint_line_mode(enum extint_line line, enum extint_line_mode mode);

/**
 * reg_extint_isr_clbk
 *
 * Registers callback function (EXTINT ISR).
 *
 * @line: EXTINT line (enum extint_line).
 * @clbk: Pointer to ISR callback function (@pend: INTR pending bits).
 */
void reg_extint_isr_clbk(enum extint_line line, BaseType_t (*clbk)(unsigned int pend));

/**
 * enable_extint_intr
 *
 * Enables interrupt for EXTINT line.
 *
 * @line: EXTINT line (enum extint_line).
 */
void enable_extint_intr(enum extint_line line);

/**
 * enable_extint_intr_uns
 *
 * Enables interrupt for EXTINT line (task unsafe).
 *
 * @line: EXTINT line (enum extint_line).
 */
inline void enable_extint_intr_uns(enum extint_line line)
{
	EXTI->IMR |= line;
}

/**
 * disable_extint_intr
 *
 * Disables interrupt for EXTINT line.
 *
 * @line: EXTINT line (enum extint_line).
 */
void disable_extint_intr(enum extint_line line);

/**
 * disable_extint_intr_uns
 *
 * Disables interrupt for EXTINT line (task unsafe).
 *
 * @line: EXTINT line (enum extint_line).
 */
inline void disable_extint_intr_uns(enum extint_line line)
{
	EXTI->IMR &= ~line;
}

/**
 * is_extint_intr_enabled
 *
 * Checks if interrupt is enabled for EXTINT line.
 *
 * @line: EXTINT line (enum extint_line).
 */
inline boolean_t is_extint_intr_enabled(enum extint_line line)
{
	return ((EXTI->IMR & line) ? TRUE : FALSE);
}

/**
 * clear_extint_intr
 *
 * Clears pending interrupt bit.
 *
 * @line: EXTINT line (enum extint_line).
 */
inline void clear_extint_intr(enum extint_line line)
{
	EXTI->PR = line;
}

#endif

#endif
