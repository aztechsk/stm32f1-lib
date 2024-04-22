/*
 * tim.c
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
#include "tim_isr.h"
#include "tim.h"

#if TIMER == 1

static void upd_psc_arr(timer dev);
static int get_clk_int(int apb_inst);

/**
 * init_timer
 */
void init_timer(timer dev, enum timer_mode mode)
{
#ifdef TIM2
	if (dev->mmio == TIM2) {
		dev->irqn = TIM2_IRQn;
                dev->apb_inst = CLK_LOW_SPEED_APB1;
                dev->apb_bmp =  RCC_APB1ENR_TIM2EN;
		goto lb1;
	}
#endif
#ifdef TIM3
	if (dev->mmio == TIM3) {
		dev->irqn = TIM3_IRQn;
                dev->apb_inst = CLK_LOW_SPEED_APB1;
                dev->apb_bmp =  RCC_APB1ENR_TIM3EN;
		goto lb1;
	}
#endif
#ifdef TIM4
	if (dev->mmio == TIM4) {
		dev->irqn = TIM4_IRQn;
                dev->apb_inst = CLK_LOW_SPEED_APB1;
                dev->apb_bmp =  RCC_APB1ENR_TIM4EN;
		goto lb1;
	}
#endif
#ifdef TIM5
	if (dev->mmio == TIM5) {
		dev->irqn = TIM5_IRQn;
                dev->apb_inst = CLK_LOW_SPEED_APB1;
                dev->apb_bmp =  RCC_APB1ENR_TIM5EN;
		goto lb1;
	}
#endif
	crit_err_exit(BAD_PARAMETER);
lb1:
	dev->mode = mode;
        upd_psc_arr(dev);
	NVIC_DisableIRQ(dev->irqn);
	enable_per_apb_clk(dev->apb_inst, dev->apb_bmp);
	dev->reg_cr1 = dev->mmio->CR1;
	dev->reg_cr1 &= ~0x03FF;
	dev->reg_cr1 |= TIM_CR1_ARPE;
	dev->reg_smcr = dev->mmio->SMCR;
	dev->reg_smcr &= ~0xFFF7;
	if (mode == TIMER_ONESHOT_ISR_MODE) {
		dev->reg_cr1 |= TIM_CR1_OPM;
	}
	dev->mmio->CR1 = dev->reg_cr1;
        dev->mmio->SMCR = dev->reg_smcr;
	dev->mmio->PSC = dev->reg_psc;
	dev->mmio->ARR = dev->reg_arr;
	dev->mmio->EGR = TIM_EGR_UG;
	dev->mmio->SR = 0;
	NVIC_SetPriority(dev->irqn, configLIBRARY_MAX_API_CALL_INTERRUPT_PRIORITY);
        NVIC_ClearPendingIRQ(dev->irqn);
	reg_tim_isr_clbk(dev->mmio, dev->isr_clbk, dev);
        NVIC_EnableIRQ(dev->irqn);
}

/**
 * reconf_timer
 */
void reconf_timer(timer dev)
{
	upd_psc_arr(dev);
        taskENTER_CRITICAL();
        dev->mmio->CR1 |= TIM_CR1_URS;
	dev->mmio->PSC = dev->reg_psc;
	dev->mmio->ARR = dev->reg_arr;
	dev->mmio->EGR = TIM_EGR_UG;
        dev->mmio->CR1 &= ~TIM_CR1_URS;
	dev->mmio->SR = 0;
        taskEXIT_CRITICAL();
}

/**
 * start_timer
 */
void start_timer(timer dev)
{
	dev->mmio->DIER = TIM_DIER_UIE;
	dev->mmio->CR1 |= TIM_CR1_CEN;
}

/**
 * restart_timer
 */
void restart_timer(timer dev)
{
	dev->mmio->CR1 |= TIM_CR1_UDIS;
	dev->mmio->EGR = TIM_EGR_UG;
        dev->mmio->CR1 &= ~TIM_CR1_UDIS;
        dev->mmio->SR = 0;
}

/**
 * stop_timer
 */
void stop_timer(timer dev)
{
	dev->mmio->DIER &= ~TIM_DIER_UIE;
	dev->mmio->CR1 &= ~TIM_CR1_CEN;
        dev->mmio->EGR = TIM_EGR_UG;
	dev->mmio->SR = 0;
}

/**
 * clear_timer_sr
 */
void clear_timer_sr(timer dev)
{
	dev->mmio->SR = 0;
}

/**
 * upd_psc_arr
 */
static void upd_psc_arr(timer dev)
{
	int clk_int, psc, arr;

#if defined(TIM2) || defined(TIM3) || defined(TIM4) || defined(TIM5)
	clk_int = get_clk_int(CLK_LOW_SPEED_APB1);
#else
	crit_err_exit(BAD_PARAMETER);
#endif
	for (psc = 1; ; psc++) {
		float us1 = (float) clk_int / psc / 1000000;
		arr = us1 * dev->tm_us - 1;
		if (arr < 65535) {
			break;
		}
	}
	psc -= 1;
	dev->reg_psc = psc;
        dev->reg_arr = arr;
}

/**
 * get_clk_int
 */
static int get_clk_int(int apb_inst)
{
	if (apb_inst == CLK_LOW_SPEED_APB1) {
		if (clk_freq.hclk / clk_freq.pclk1 > 1) {
			return (clk_freq.pclk1 * 2);
		} else {
			return (clk_freq.pclk1);
		}
	} else {
		if (clk_freq.hclk / clk_freq.pclk2 > 1) {
			return (clk_freq.pclk2 * 2);
		} else {
			return (clk_freq.pclk2);
		}
	}
}

#endif
