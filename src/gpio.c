/*
 * gpio.c
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
#include "atom.h"
#include "msgconf.h"
#include "criterr.h"
#include "clk.h"
#include "gpio.h"
#include <stdarg.h>

extern inline boolean_t get_gpio_lev(enum gpio_pin pin, GPIO_TypeDef *port);
extern inline void set_gpio_lev(enum gpio_pin pin, GPIO_TypeDef *port, boolean_t lev);
extern inline boolean_t get_gpio_out(enum gpio_pin pin, GPIO_TypeDef *port);

/**
 * init_gpio_port
 */
void init_gpio_port(GPIO_TypeDef *port)
{
	unsigned int bmp = 0;

	if (port == GPIOA) {
		bmp =  RCC_APB2ENR_IOPAEN;
#ifdef GPIOB
	} else if (port == GPIOB) {
		bmp =  RCC_APB2ENR_IOPBEN;
#endif
#ifdef GPIOC
	} else if (port == GPIOC) {
		bmp =  RCC_APB2ENR_IOPCEN;
#endif
#ifdef GPIOD
	} else if (port == GPIOD) {
		bmp =  RCC_APB2ENR_IOPDEN;
#endif
#ifdef GPIOE
	} else if (port == GPIOE) {
		bmp =  RCC_APB2ENR_IOPEEN;
#endif
#ifdef GPIOF
	} else if (port == GPIOF) {
		bmp =  RCC_APB2ENR_IOPFEN;
#endif
#ifdef GPIOG
	} else if (port == GPIOG) {
		bmp =  RCC_APB2ENR_IOPGEN;
#endif
#ifdef GPIOH
	} else if (port == GPIOH) {
		bmp =  RCC_APB2ENR_IOPHEN;
#endif
	} else {
		crit_err_exit(BAD_PARAMETER);
	}
	enable_per_apb_clk(CLK_HIGH_SPEED_APB2, bmp);
}

/**
 * disable_gpio_port
 */
void disable_gpio_port(GPIO_TypeDef *port)
{
	unsigned int bmp = 0;

	if (port == GPIOA) {
		bmp =  RCC_APB2ENR_IOPAEN;
#ifdef GPIOB
	} else if (port == GPIOB) {
		bmp =  RCC_APB2ENR_IOPBEN;
#endif
#ifdef GPIOC
	} else if (port == GPIOC) {
		bmp =  RCC_APB2ENR_IOPCEN;
#endif
#ifdef GPIOD
	} else if (port == GPIOD) {
		bmp =  RCC_APB2ENR_IOPDEN;
#endif
#ifdef GPIOE
	} else if (port == GPIOE) {
		bmp =  RCC_APB2ENR_IOPEEN;
#endif
#ifdef GPIOF
	} else if (port == GPIOF) {
		bmp =  RCC_APB2ENR_IOPFEN;
#endif
#ifdef GPIOG
	} else if (port == GPIOG) {
		bmp =  RCC_APB2ENR_IOPGEN;
#endif
#ifdef GPIOH
	} else if (port == GPIOH) {
		bmp =  RCC_APB2ENR_IOPHEN;
#endif
	} else {
		crit_err_exit(BAD_PARAMETER);
	}
	disable_per_apb_clk(CLK_HIGH_SPEED_APB2, bmp);
}

/**
 * gpio_conf
 */
void conf_gpio(enum gpio_pin pin, GPIO_TypeDef *port, enum gpio_pin_func func, ...)
{
	volatile uint32_t *creg;
	int pos = 0;
	unsigned int m = pin, cfg = 0;
        va_list ap;
        enum gpio_pin_feat feat, pul = GPIO_FEAT_END, lev = GPIO_FEAT_END;

	if (pin > GPIO_PIN7) {
		creg = &port->CRH;
		m >>= 8;
	} else {
		creg = &port->CRL;
	}
	while (!(m & 1)) {
		pos++;
		m >>= 1;
	}
	pos *= 4;
	va_start(ap, func);
	while ((feat = va_arg(ap, int)) != GPIO_FEAT_END) {
		switch (feat) {
		case GPIO_FEAT_PULL_UP    :
			/* FALLTHRU */
		case GPIO_FEAT_PULL_DOWN  :
			pul = feat;
			break;
		case GPIO_FEAT_LEV_0      :
			/* FALLTHRU */
		case GPIO_FEAT_LEV_1      :
			lev = feat;
			break;
		case GPIO_FEAT_SPEED_LOW  :
			cfg = 2;
			break;
		case GPIO_FEAT_SPEED_NORM :
			cfg = 1;
			break;
		case GPIO_FEAT_SPEED_HIGH :
			cfg = 3;
			break;
		default                   :
			crit_err_exit(BAD_PARAMETER);
			break;
		}
	}
	va_end(ap);
	switch (func) {
	case GPIO_FUNC_INPUT :
		taskENTER_CRITICAL();
                if (pul == GPIO_FEAT_PULL_UP) {
			port->ODR |= pin;
		} else if (pul == GPIO_FEAT_PULL_DOWN) {
			port->ODR &= ~pin;
		}
		if (pul == GPIO_FEAT_END) {
			cfg = 1 << 2;
		} else {
			cfg = 1 << 3;
		}
                *creg = (*creg & ~(0xF << pos)) | (cfg << pos);
		taskEXIT_CRITICAL();
		break;
	case GPIO_FUNC_ANALOG :
		taskENTER_CRITICAL();
		*creg &= ~(0xF << pos);
		taskEXIT_CRITICAL();
		break;
	case GPIO_FUNC_OUTPUT_DRAIN :
		cfg |= 1 << 2;
		/* FALLTHRU */
	case GPIO_FUNC_OUTPUT :
		if (!cfg) {
			crit_err_exit(BAD_PARAMETER);
		}
                taskENTER_CRITICAL();
                if (lev == GPIO_FEAT_LEV_1) {
			port->ODR |= pin;
		} else if (lev == GPIO_FEAT_LEV_0) {
			port->ODR &= ~pin;
		} else {
			taskEXIT_CRITICAL();
			crit_err_exit(BAD_PARAMETER);
		}
		*creg = (*creg & ~(0xF << pos)) | (cfg << pos);
                taskEXIT_CRITICAL();
		break;
	case GPIO_FUNC_PERIPH_DRAIN :
		cfg |= 1 << 2;
		/* FALLTHRU */
        case GPIO_FUNC_PERIPH :
        	if (!cfg) {
			crit_err_exit(BAD_PARAMETER);
		}
		cfg |= 1 << 3;
		taskENTER_CRITICAL();
		*creg = (*creg & ~(0xF << pos)) | (cfg << pos);
		taskEXIT_CRITICAL();
		break;
	default :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
}
