/*
 * wwdg.c
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
#include "wwdg.h"

static unsigned int cr;

/**
 * init_wwdg
 */
void init_wwdg(int cnt_top, int win_top, enum wwdg_div div)
{
	if (cnt_top & ~0x7F || !(cnt_top & 0x40)) {
		crit_err_exit(BAD_PARAMETER);
	}
	if (win_top > cnt_top) {
		crit_err_exit(BAD_PARAMETER);
	}
        cr = cnt_top;
	enable_per_apb_clk(CLK_LOW_SPEED_APB1, RCC_APB1ENR_WWDGEN);
        WWDG->CFR = (div << WWDG_CFR_WDGTB_Pos) | win_top;
	WWDG->CR = WWDG_CR_WDGA | cnt_top;
}

/**
 * reload_wwdg
 */
void reload_wwdg(void)
{
	WWDG->CR = cr;
}
