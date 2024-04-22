/*
 * afio.h
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

#ifndef AFIO_H
#define AFIO_H

/**
 * init_afio
 *
 * Configures AFIO remap registers.
 */
void init_afio(void);

/**
 * disable_afio
 *
 * Disables AFIO module clock.
 */
void disable_afio(void);

/**
 * enable_afio
 *
 * Enables AFIO module clock.
 */
void enable_afio(void);

/**
 * set_afio_evcr
 *
 * Remap peripheral in AFIO EVCR register.
 *
 * @mask: Mask to clear peripheral bits.
 * @bits: Bits to set peripheral mapping.
 */
void set_afio_evcr(unsigned int mask, unsigned int bits);

/**
 * set_afio_mapr
 *
 * Remap peripheral in AFIO MAPR register.
 *
 * @mask: Mask to clear peripheral bits.
 * @bits: Bits to set peripheral mapping.
 *   MAPR_SWJ_CFG must be defined in board.h file.
 */
void set_afio_mapr(unsigned int mask, unsigned int bits);

/**
 * set_afio_mapr2
 *
 * Remap peripheral in AFIO MAPR2 register.
 *
 * @mask: Mask to clear peripheral bits.
 * @bits: Bits to set peripheral mapping.
 */
void set_afio_mapr2(unsigned int mask, unsigned int bits);

#endif
