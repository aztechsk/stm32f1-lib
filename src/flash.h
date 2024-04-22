/*
 * flash.h
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

#ifndef FLASH_H
#define FLASH_H

enum flash_user_rst_stdby {
	FLASH_USER_RST_STDBY_ON,
        FLASH_USER_RST_STDBY_OFF
};

enum flash_user_rst_stop {
	FLASH_USER_RST_STOP_ON,
	FLASH_USER_RST_STOP_OFF
};

enum flash_user_wdg {
	FLASH_USER_WDG_HW,
        FLASH_USER_WDG_SW
};

struct flash_opt_bytes {
	enum flash_user_rst_stdby rst_stdby; // Reset event when entering stop mode.
        enum flash_user_rst_stop rst_stop;  // Reset event when entering standby mode.
        enum flash_user_wdg wdg; // Selects WATCHDOG event (hardware or software).
	boolean_t rdp; // Enables read protection of code stored in FLASH.
	uint8_t data0; // User data byte 0.
	uint8_t data1; // User data byte 1.
	uint32_t wrp;  // Bitmap of write protected FLASH pages.
};

/**
 * set_flash_wait_states
 *
 * Sets FLASH wait states.
 *
 * @sysclk: SYSCLK frequency.
 */
void set_flash_wait_states(int sysclk);

/**
 * read_flash_opt_bytes
 *
 * Reads OPTION bytes from FLASH.
 *
 * @ob: Pointer to struct flash_opt_bytes (read result).
 *
 * Returns: TRUE - success; FALSE - OPTION bytes corrupted.
 */
boolean_t read_flash_opt_bytes(struct flash_opt_bytes *ob);

/**
 * write_flash_opt_bytes
 *
 * Writes new values of OPTION bytes to FLASH.
 *
 * @ob: Pointer to struct flash_opt_bytes (new values).
 *
 * Returns: TRUE - success; FALSE - verification of write failed.
 */
boolean_t write_flash_opt_bytes(struct flash_opt_bytes *ob);

#if TERMOUT == 1
/**
 * log_flash_opt_bytes
 *
 * Prints OPTION bytes to console.
 *
 * @ob: Pointer to struct flash_opt_bytes (for print).
 */
void log_flash_opt_bytes(struct flash_opt_bytes *ob);
#endif

#endif
