/*
 * i2c_isr.h
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

#ifndef I2C_ISR_H
#define I2C_ISR_H

#ifndef I2CM
 #define I2CM 0
#endif

#if I2CM == 1

/**
 * reg_i2c_isr_clbks
 */
void reg_i2c_isr_clbks(I2C_TypeDef *id, BaseType_t (*cfn_ev)(void *), BaseType_t (*cfn_er)(void *),
		       void *dev);

/**
 * reg_i2c_ev_isr_clbk
 */
void reg_i2c_ev_isr_clbk(I2C_TypeDef *id, BaseType_t (*cfn)(void *));

/**
 * get_i2c_ev_isr_clbk
 */
BaseType_t (*get_i2c_ev_isr_clbk(I2C_TypeDef *id))(void *);

#endif

#endif
