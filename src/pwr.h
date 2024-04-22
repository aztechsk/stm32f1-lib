/*
 * pwr.h
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

#ifndef PWR_H
#define PWR_H

/**
 * init_pwr
 *
 * Enables Power control peripheral.
 */
void init_pwr(void);

/**
 * enable_bkp_domain_write
 *
 * Enables write to Backup domain registers.
 */
void enable_bkp_domain_write(void);

enum voltage_threshold {
	VOLTAGE_THRESHOLD_2_2V,
	VOLTAGE_THRESHOLD_2_3V,
	VOLTAGE_THRESHOLD_2_4V,
	VOLTAGE_THRESHOLD_2_5V,
	VOLTAGE_THRESHOLD_2_6V,
	VOLTAGE_THRESHOLD_2_7V,
	VOLTAGE_THRESHOLD_2_8V,
        VOLTAGE_THRESHOLD_2_9V
};

/**
 * enable_voltage_detector
 *
 * @thres: Voltage threshold (enum voltage_threshold).
 */
void enable_voltage_detector(enum voltage_threshold t);

/**
 * disable_voltage_detector
 */
void disable_voltage_detector(void);

#endif
