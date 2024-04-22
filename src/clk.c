/*
 * clk.c
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

struct clk_freq clk_freq = {.hsi = 8000000, .lsi = 40000};

/**
 * enable_hse
 */
void enable_hse(enum ext_osc_mode mode)
{
	switch (mode) {
	case EXT_OSC_CRYSTAL :
		break;
	case EXT_OSC_BYPASS  :
		RCC->CR |= RCC_CR_HSEBYP;
		break;
	default		     :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
        RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY));
}

/**
 * disable_hse
 */
void disable_hse(void)
{
	RCC->CR &= ~RCC_CR_HSEON;
        RCC->CR &= ~RCC_CR_HSEBYP;
	while (RCC->CR & RCC_CR_HSERDY);
}

/**
 * enable_hsi
  */
void enable_hsi(void)
{
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY));
}

/**
 * disable_hsi
 */
void disable_hsi(void)
{
	RCC->CR &= ~RCC_CR_HSION;
        while (RCC->CR & RCC_CR_HSIRDY);
}

/**
 * is_hsi_enabled
 */
boolean_t is_hsi_enabled(void)
{
	if (RCC->CR & RCC_CR_HSION) {
		return (TRUE);
	} else {
		return (FALSE);
	}
}

/**
 * enable_lse
 */
void enable_lse(enum ext_osc_mode mode)
{
	if (!(PWR->CR & PWR_CR_DBP)) {
		PWR->CR |= PWR_CR_DBP;
		while (!(PWR->CR & PWR_CR_DBP));
	}
	switch (mode) {
	case EXT_OSC_CRYSTAL :
		break;
	case EXT_OSC_BYPASS  :
		RCC->BDCR |= RCC_BDCR_LSEBYP;
		break;
	default		     :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
        RCC->BDCR |= RCC_BDCR_LSEON;
	while (!(RCC->BDCR & RCC_BDCR_LSERDY));
}

/**
 * disable_lse
 */
void disable_lse(void)
{
	if (!(PWR->CR & PWR_CR_DBP)) {
		PWR->CR |= PWR_CR_DBP;
		while (!(PWR->CR & PWR_CR_DBP));
	}
	RCC->BDCR &= ~RCC_BDCR_LSEON;
	RCC->BDCR &= ~RCC_BDCR_LSEBYP;
	while (RCC->BDCR & RCC_BDCR_LSERDY);
}

/**
 * enable_lsi
 */
void enable_lsi(void)
{
	RCC->CSR |= RCC_CSR_LSION;
	while (!(RCC->CSR & RCC_CSR_LSIRDY));
}

/**
 * disable_lsi
 */
void disable_lsi(void)
{
	RCC->CSR &= ~RCC_CSR_LSION;
        while (RCC->CSR & RCC_CSR_LSIRDY);
}

/**
 * enable_pll
 */
void enable_pll(enum pll_input_clk iclk, unsigned int mul)
{
	unsigned int tmp;

	tmp = RCC->CFGR;
        tmp &= ~(RCC_CFGR_PLLMULL_Msk | RCC_CFGR_PLLXTPRE_Msk | RCC_CFGR_PLLSRC_Msk);
	tmp |= mul;
	switch (iclk) {
	case PLL_INPUT_CLK_HSI_DIV_2 :
		break;
	case PLL_INPUT_CLK_HSE_DIV_2 :
		tmp |= RCC_CFGR_PLLXTPRE;
		/* FALLTHRU */
        case PLL_INPUT_CLK_HSE       :
		tmp |= RCC_CFGR_PLLSRC;
		break;
	default                      :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
        RCC->CFGR = tmp;
	RCC->CR |= RCC_CR_PLLON;
        while (!(RCC->CR & RCC_CR_PLLRDY));
}

/**
 * disable_pll
 */
void disable_pll(void)
{
	RCC->CR &= ~RCC_CR_PLLON;
	while (RCC->CR & RCC_CR_PLLRDY);
}

/**
 * select_sysclk_src
 */
void select_sysclk_src(enum sysclk_src src)
{
	unsigned int tmp;

	tmp = RCC->CFGR;
	tmp &= ~RCC_CFGR_SW_Msk;
	switch (src) {
	case SYSCLK_SRC_HSI :
		break;
	case SYSCLK_SRC_HSE :
		/* FALLTHRU */
	case SYSCLK_SRC_PLL :
		tmp |= src;
		break;
	default             :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
	RCC->CFGR = tmp;
	while (((RCC->CFGR & RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos) != src);
}

/**
 * set_ahb_div
 */
void set_ahb_div(enum sysclk_ahb_div div)
{
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE_Msk) | (div << RCC_CFGR_HPRE_Pos);
}

/**
 * set_apb_div
 */
void set_apb_div(enum clk_apb_inst_sel apb, enum hclk_apb_div div)
{
	switch (apb) {
	case CLK_LOW_SPEED_APB1  :
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1_Msk) | (div << RCC_CFGR_PPRE1_Pos);
		break;
	case CLK_HIGH_SPEED_APB2 :
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2_Msk) | (div << RCC_CFGR_PPRE2_Pos);
		break;
	default                  :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
}

/**
 * enable_per_ahb_clk
 */
void enable_per_ahb_clk(unsigned int bmp)
{
	unsigned int dummy;

	taskENTER_CRITICAL();
	RCC->AHBENR |= bmp;
        dummy = RCC->AHBENR;
        taskEXIT_CRITICAL();
}

/**
 * disable_per_ahb_clk
 */
void disable_per_ahb_clk(unsigned int bmp)
{
	unsigned int dummy;

	taskENTER_CRITICAL();
	RCC->AHBENR &= ~bmp;
        dummy = RCC->AHBENR;
        taskEXIT_CRITICAL();
}

/**
 * enable_per_apb_clk
 */
void enable_per_apb_clk(enum clk_apb_inst_sel apb, unsigned int bmp)
{
	unsigned int dummy;

	switch (apb) {
	case CLK_LOW_SPEED_APB1  :
		taskENTER_CRITICAL();
                RCC->APB1ENR |= bmp;
		dummy = RCC->APB1ENR;
		taskEXIT_CRITICAL();
		break;
	case CLK_HIGH_SPEED_APB2 :
		taskENTER_CRITICAL();
                RCC->APB2ENR |= bmp;
		dummy = RCC->APB2ENR;
		taskEXIT_CRITICAL();
		break;
	default                  :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
}

/**
 * disable_per_apb_clk
 */
void disable_per_apb_clk(enum clk_apb_inst_sel apb, unsigned int bmp)
{
	unsigned int dummy;

	switch (apb) {
	case CLK_LOW_SPEED_APB1  :
		taskENTER_CRITICAL();
                RCC->APB1ENR &= ~bmp;
		dummy = RCC->APB1ENR;
		taskEXIT_CRITICAL();
		break;
	case CLK_HIGH_SPEED_APB2 :
		taskENTER_CRITICAL();
                RCC->APB2ENR &= ~bmp;
		dummy = RCC->APB2ENR;
		taskEXIT_CRITICAL();
		break;
	default                  :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
}
