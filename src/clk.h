/*
 * clk.h
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

#ifndef CLK_H
#define CLK_H

struct clk_freq {
	int hse;
	int lse;
	int hsi;
	int lsi;
	int sys;
	int hclk;
	int pclk1;
	int pclk2;
};

extern struct clk_freq clk_freq;

enum ext_osc_mode {
	EXT_OSC_CRYSTAL,
        EXT_OSC_BYPASS
};

/**
 * enable_hse
 *
 * Enables high speed external clock.
 *
 * @mode: enum ext_osc_mode.
 */
void enable_hse(enum ext_osc_mode mode);

/**
 * disable_hse
 *
 * Disables high speed external clock.
 */
void disable_hse(void);

/**
 * enable_hsi
 *
 * Enables high speed internal clock.
 */
void enable_hsi(void);

/**
 * disable_hsi
 *
 * Disables high speed internal clock.
 */
void disable_hsi(void);

/**
 * is_hsi_enabled
 *
 * Check if high speed internal clock is enabled.
 *
 * Returns: boolean.
 */
boolean_t is_hsi_enabled(void);

/**
 * enable_lse
 *
 * Enables low speed external clock.
 *
 * @mode: enum ext_osc_mode.
 */
void enable_lse(enum ext_osc_mode mode);

/**
 * disable_lse
 *
 * Disables low speed external clock.
 */
void disable_lse(void);

/**
 * enable_lsi
 *
 * Enables low speed internal clock.
 */
void enable_lsi(void);

/**
 * disable_lsi
 *
 * Disables low speed internal clock.
 */
void disable_lsi(void);

enum pll_input_clk {
	PLL_INPUT_CLK_HSI_DIV_2,
        PLL_INPUT_CLK_HSE,
        PLL_INPUT_CLK_HSE_DIV_2
};

/**
 * enable_pll
 *
 * Enables PLL.
 *
 * @iclk: PLL input clock (enum pll_input_clk).
 * @mul: PLL multiplication factor (RCC_CFGR_PLLMULLX).
 */
void enable_pll(enum pll_input_clk iclk, unsigned int mul);

/**
 * disable_pll
 *
 * Disables PLL.
 */
void disable_pll(void);

enum sysclk_src {
	SYSCLK_SRC_HSI,
        SYSCLK_SRC_HSE,
        SYSCLK_SRC_PLL
};

/**
 * select_sysclk_src
 *
 * Selects SYSCLOCK source.
 *
 * @src: SYSCLOCK source (enum sysclk_src).
 */
void select_sysclk_src(enum sysclk_src src);

enum sysclk_ahb_div {
	SYSCLK_AHB_DIV_1,
        SYSCLK_AHB_DIV_2 = 8,
        SYSCLK_AHB_DIV_4,
        SYSCLK_AHB_DIV_8,
        SYSCLK_AHB_DIV_16,
        SYSCLK_AHB_DIV_64,
        SYSCLK_AHB_DIV_128,
        SYSCLK_AHB_DIV_256,
        SYSCLK_AHB_DIV_512
};

/**
 * set_ahb_div
 *
 * Sets AHB (HCLK) clock frequency (SYSCLOCK / divider).
 *
 * @div: SYSCLOCK divider (enum sysclk_ahb_div).
 */
void set_ahb_div(enum sysclk_ahb_div div);

enum clk_apb_inst_sel {
	CLK_LOW_SPEED_APB1,
	CLK_HIGH_SPEED_APB2
};

enum hclk_apb_div {
	HCLK_APB_DIV_1,
        HCLK_APB_DIV_2 = 4,
        HCLK_APB_DIV_4,
        HCLK_APB_DIV_8,
        HCLK_APB_DIV_16
};

/**
 * set_apb_div
 *
 * Sets APB (PCLK) clock frequency (HCLK / divider).
 *
 * @apb: APB instance selection (enum clk_apb_inst_sel).
 * @div: HCLK divider (enum hclk_apb_div).
 */
void set_apb_div(enum clk_apb_inst_sel apb, enum hclk_apb_div div);

/**
 * enable_per_ahb_clk
 *
 * Enables peripheral AHB clock.
 *
 * @bmp: Peripheral bitmap (RCC_AHBENR_*EN).
 */
void enable_per_ahb_clk(unsigned int bmp);

/**
 * disable_per_ahb_clk
 *
 * Disables peripheral AHB clock.
 *
 * @bmp: Peripheral bitmap (RCC_AHBENR_*EN).
 */
void disable_per_ahb_clk(unsigned int bmp);

/**
 * enable_per_apb_clk
 *
 * Enables peripheral APB clock.
 *
 * @apb: APB instance selection (enum clk_apb_inst_sel).
 * @bmp: Peripheral bitmap (RCC_APB1ENR_*EN or RCC_APB2ENR_*EN).
 */
void enable_per_apb_clk(enum clk_apb_inst_sel apb, unsigned int bmp);

/**
 * disable_per_apb_clk
 *
 * Disables peripheral APB clock.
 *
 * @apb: APB instance selection (enum clk_apb_inst_sel).
 * @bmp: Peripheral bitmap (RCC_APB1ENR_*EN or RCC_APB2ENR_*EN).
 */
void disable_per_apb_clk(enum clk_apb_inst_sel apb, unsigned int bmp);

#endif
