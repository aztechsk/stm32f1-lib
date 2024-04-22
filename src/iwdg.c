/*
 * iwdg.c
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
#include "iwdg.h"

extern inline void reload_iwdg(void);

/**
 * init_iwdg
 */
void init_iwdg(int ms)
{
	int psc, rlr;

	for (psc = 4; ; psc *= 2) {
		float us1 = (float) 40000 / psc / 1000;
		rlr = us1 * ms - 1;
		if (rlr < 4096) {
			break;
		}
	}
        IWDG->KR = 0xCCCC;
        IWDG->KR = 0x5555;
        IWDG->RLR = rlr;
	while (IWDG->SR & IWDG_SR_RVU);
	int n = 4, i;
	for (i = 0; i < 7; i++) {
		if (psc == n) {
			break;
		}
		n *= 2;
	}
	if (i == 7) {
		crit_err_exit(BAD_PARAMETER);
	}
        IWDG->PR = i;
	while (IWDG->SR & IWDG_SR_PVU);
	IWDG->KR = 0xAAAA;
}
