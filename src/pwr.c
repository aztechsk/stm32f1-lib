/*
 * pwr.c
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
#include "pwr.h"

/**
 * init_pwr
 */
void init_pwr(void)
{
	enable_per_apb_clk(CLK_LOW_SPEED_APB1, RCC_APB1ENR_PWREN);
}

/**
 * enable_bkp_domain_write
 */
void enable_bkp_domain_write(void)
{
	PWR->CR |= PWR_CR_DBP;
	while (!(PWR->CR & PWR_CR_DBP));
}

/**
 * enable_voltage_detector
 */
void enable_voltage_detector(enum voltage_threshold t)
{
	PWR->CR = (PWR->CR & ~PWR_CR_PLS_Msk) | (t << PWR_CR_PLS_Pos);
        PWR->CR |= PWR_CR_PVDE;
}

/**
 * disable_voltage_detector
 */
void disable_voltage_detector(void)
{
	PWR->CR &= ~PWR_CR_PVDE;
}
