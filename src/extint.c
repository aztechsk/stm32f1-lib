/*
 * extint.c
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

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <gentyp.h>
#include "sysconf.h"
#include "board.h"
#include <mmio.h>
#include "msgconf.h"
#include "criterr.h"
#include "tools.h"
#include "clk.h"
#include "extint.h"

#if EXTINT == 1

extern inline void enable_extint_intr_uns(enum extint_line line);
extern inline void disable_extint_intr_uns(enum extint_line line);
extern inline boolean_t is_extint_intr_enabled(enum extint_line line);
extern inline void clear_extint_intr(enum extint_line line);

static BaseType_t (*clbk0)(unsigned int);
static BaseType_t (*clbk1)(unsigned int);
static BaseType_t (*clbk2)(unsigned int);
static BaseType_t (*clbk3)(unsigned int);
static BaseType_t (*clbk4)(unsigned int);
static BaseType_t (*clbk9_5[EXTINT_9_5_CLBK_ARY_SIZE])(unsigned int);
static BaseType_t (*clbk15_10[EXTINT_15_10_CLBK_ARY_SIZE])(unsigned int);

/**
 * init_extint_intr
 */
void init_extint_intr(enum extint_line line)
{
	int irqn;

	if (line == EXTINT_LINE0) {
		irqn = EXTI0_IRQn;
	} else if (line == EXTINT_LINE1) {
		irqn = EXTI1_IRQn;
	} else if (line == EXTINT_LINE2) {
		irqn = EXTI2_IRQn;
	} else if (line == EXTINT_LINE3) {
		irqn = EXTI3_IRQn;
        } else if (line == EXTINT_LINE4) {
		irqn = EXTI4_IRQn;
	} else if (bit_pos(line) < 10) {
		irqn = EXTI9_5_IRQn;
	} else if (bit_pos(line) < 16) {
		irqn = EXTI15_10_IRQn;
	} else {
		crit_err_exit(BAD_PARAMETER);
	}
        NVIC_DisableIRQ(irqn);
        NVIC_SetPriority(irqn, configLIBRARY_MAX_API_CALL_INTERRUPT_PRIORITY);
        NVIC_ClearPendingIRQ(irqn);
	NVIC_EnableIRQ(irqn);
}

/**
 * is_extint_intr_init
 */
boolean_t is_extint_intr_init(enum extint_line line)
{
	int irqn;

	if (line == EXTINT_LINE0) {
		irqn = EXTI0_IRQn;
	} else if (line == EXTINT_LINE1) {
		irqn = EXTI1_IRQn;
	} else if (line == EXTINT_LINE2) {
		irqn = EXTI2_IRQn;
	} else if (line == EXTINT_LINE3) {
		irqn = EXTI3_IRQn;
        } else if (line == EXTINT_LINE4) {
		irqn = EXTI4_IRQn;
	} else if (bit_pos(line) < 10) {
		irqn = EXTI9_5_IRQn;
	} else if (bit_pos(line) < 16) {
		irqn = EXTI15_10_IRQn;
	} else {
		crit_err_exit(BAD_PARAMETER);
	}
	if (NVIC_GetEnableIRQ(irqn)) {
		return (TRUE);
	} else {
		return (FALSE);
	}
}

/**
 * map_extint_line
 */
void map_extint_line(enum extint_line line, GPIO_TypeDef *port)
{
	unsigned int tmp;
	int pn = 0;
	int lnum = bit_pos(line);
        int r_idx = lnum / 4;
	int shft = (lnum - (r_idx * 4)) * 4;

	if (port == GPIOA) {
		pn = 0;
#ifdef GPIOB
	} else if (port == GPIOB) {
		pn = 1;
#endif
#ifdef GPIOC
	} else if (port == GPIOC) {
		pn = 2;
#endif
#ifdef GPIOD
	} else if (port == GPIOD) {
		pn = 3;
#endif
#ifdef GPIOE
	} else if (port == GPIOE) {
		pn = 4;
#endif
#ifdef GPIOF
	} else if (port == GPIOF) {
		pn = 5;
#endif
#ifdef GPIOG
        } else if (port == GPIOG) {
		pn = 6;
#endif
	} else {
		crit_err_exit(BAD_PARAMETER);
	}
        taskENTER_CRITICAL();
        tmp = AFIO->EXTICR[r_idx] & ~(0x0F << shft);
	AFIO->EXTICR[r_idx] = tmp | (pn << shft);
	taskEXIT_CRITICAL();
}

/**
 * set_extint_line_mode
 */
void set_extint_line_mode(enum extint_line line, enum extint_line_mode mode)
{
	taskENTER_CRITICAL();
	EXTI->FTSR &= ~line;
        EXTI->RTSR &= ~line;
	switch (mode) {
	case EXTINT_LINE_FALL_TRIG :
		EXTI->FTSR |= line;
                taskEXIT_CRITICAL();
		break;
	case EXTINT_LINE_RISE_TRIG :
		EXTI->RTSR |= line;
                taskEXIT_CRITICAL();
		break;
	case EXTINT_LINE_EDGE_TRIG :
		EXTI->FTSR |= line;
                EXTI->RTSR |= line;
                taskEXIT_CRITICAL();
		break;
	default :
		taskEXIT_CRITICAL();
		crit_err_exit(BAD_PARAMETER);
		break;
	}
}

/**
 * reg_extint_isr_clbk
 */
void reg_extint_isr_clbk(enum extint_line line, BaseType_t (*clbk)(unsigned int pend))
{
	if (line == EXTINT_LINE0) {
		clbk0 = clbk;
	} else if (line == EXTINT_LINE1) {
		clbk1 = clbk;
	} else if (line == EXTINT_LINE2) {
		clbk2 = clbk;
	} else if (line == EXTINT_LINE3) {
		clbk3 = clbk;
        } else if (line == EXTINT_LINE4) {
		clbk4 = clbk;
	} else if (bit_pos(line) < 10) {
		taskENTER_CRITICAL();
		for (int i = 0; i < EXTINT_9_5_CLBK_ARY_SIZE; i++) {
			if (!clbk9_5[i]) {
				clbk9_5[i] = clbk;
				taskEXIT_CRITICAL();
				return;
			} else {
				if (clbk9_5[i] == clbk) {
					taskEXIT_CRITICAL();
					return;
				}
			}
		}
		taskEXIT_CRITICAL();
		crit_err_exit(UNEXP_PROG_STATE);
	} else if (bit_pos(line) < 16) {
		taskENTER_CRITICAL();
		for (int i = 0; i < EXTINT_15_10_CLBK_ARY_SIZE; i++) {
			if (!clbk15_10[i]) {
				clbk15_10[i] = clbk;
				taskEXIT_CRITICAL();
				return;
			} else {
				if (clbk15_10[i] == clbk) {
					taskEXIT_CRITICAL();
					return;
				}
			}
		}
		taskEXIT_CRITICAL();
		crit_err_exit(UNEXP_PROG_STATE);
	} else {
		crit_err_exit(BAD_PARAMETER);
	}
}

/**
 * enable_extint_intr
 */
void enable_extint_intr(enum extint_line line)
{
	taskENTER_CRITICAL();
	EXTI->IMR |= line;
	taskEXIT_CRITICAL();
}

/**
 * disable_extint_intr
 */
void disable_extint_intr(enum extint_line line)
{
	taskENTER_CRITICAL();
	EXTI->IMR &= ~line;
	taskEXIT_CRITICAL();
}

/**
 * EXTI0_IRQHandler
 */
void EXTI0_IRQHandler(void);
void EXTI0_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk0)(EXTI->PR));
}

/**
 * EXTI1_IRQHandler
 */
void EXTI1_IRQHandler(void);
void EXTI1_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk1)(EXTI->PR));
}

/**
 * EXTI2_IRQHandler
 */
void EXTI2_IRQHandler(void);
void EXTI2_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk2)(EXTI->PR));
}

/**
 * EXTI3_IRQHandler
 */
void EXTI3_IRQHandler(void);
void EXTI3_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk3)(EXTI->PR));
}

/**
 * EXTI4_IRQHandler
 */
void EXTI4_IRQHandler(void);
void EXTI4_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk4)(EXTI->PR));
}

/**
 * EXTI9_5_IRQHandler
 */
void EXTI9_5_IRQHandler(void);
void EXTI9_5_IRQHandler(void)
{
	BaseType_t tsk_wkn = pdFALSE;
	unsigned int pr = EXTI->PR;

	for (int i = 0; i < EXTINT_9_5_CLBK_ARY_SIZE; i++) {
		if (clbk9_5[i]) {
			if (pdTRUE == clbk9_5[i](pr)) {
				tsk_wkn = pdTRUE;
			}
			continue;
		}
		break;
	}
	portEND_SWITCHING_ISR(tsk_wkn);
}

/**
 * EXTI15_10_IRQHandler
 */
void EXTI15_10_IRQHandler(void);
void EXTI15_10_IRQHandler(void)
{
	BaseType_t tsk_wkn = pdFALSE;
	unsigned int pr = EXTI->PR;

	for (int i = 0; i < EXTINT_15_10_CLBK_ARY_SIZE; i++) {
		if (clbk15_10[i]) {
			if (pdTRUE == clbk15_10[i](pr)) {
				tsk_wkn = pdTRUE;
			}
			continue;
		}
		break;
	}
	portEND_SWITCHING_ISR(tsk_wkn);
}

#endif
