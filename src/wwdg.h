/*
 * wwdg.h
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

#ifndef WWDG_H
#define WWDG_H

enum wwdg_div {
	WWDG_DIV1,
	WWDG_DIV2,
	WWDG_DIV4,
	WWDG_DIV8
};

/**
 * init_wwdg
 *
 * Configures and starts WWDG.
 *
 * @cnt_top: Countdown start value (max 0x7F).
 * @win_top: Reload enable window top value (max 0x7F).
 * @div: WWDG counter clock = (PCLK1 / 4096) / div.
 */
void init_wwdg(int cnt_top, int win_top, enum wwdg_div div);

/**
 * reload_wwdg
 *
 * Reloads WWDG counter.
 */
void reload_wwdg(void);

#endif
