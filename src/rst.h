/*
 * rst.h
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

#ifndef RST_H
#define RST_H

enum rst_apb_inst_sel {
	RST_LOW_SPEED_APB1,
	RST_HIGH_SPEED_APB2
};

/**
 * reset_peripheral
 *
 * Resets peripheral.
 *
 * @apb: APB instance selection (enum rst_apb_inst_sel).
 * @bmp: Peripheral bitmap (RCC_APB1RSTR_*RST or RCC_APB2RSTR_*RST).
 */
void reset_peripheral(enum rst_apb_inst_sel apb, unsigned int bmp);

/**
 * reset_backup_domain
 *
 * Resets backup domain.
 */
void reset_backup_domain(void);

enum reset_cause {
	RESET_CAUSE_LOW_PWR,
        RESET_CAUSE_WIN_WDT,
        RESET_CAUSE_IND_WDT,
        RESET_CAUSE_SOFT,
        RESET_CAUSE_POR_PDR,
        RESET_CAUSE_NRST,
	RESET_CAUSE_ERR
};

/**
 * get_reset_cause
 *
 * Returns reason of last reset. Clear reset cause bits.
 */
enum reset_cause get_reset_cause(void);

#if TERMOUT == 1
/**
 * get_reset_cause_str
 *
 * Returns reason of reset as string.
 *
 * @cause: enum reset_cause.
 */
const char *get_reset_cause_str(enum reset_cause cause);
#endif

/**
 * reset_request
 *
 * Resets microcontroller.
 */
void reset_request(char *msg);

#endif
