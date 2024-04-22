/*
 * tim_isr.h
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

#ifndef TIM_ISR_H
#define TIM_ISR_H

/**
 * reg_tim_isr_clbk
 */
void reg_tim_isr_clbk(TIM_TypeDef *id, BaseType_t (*cfn)(void *), void *dev);

/**
 * reg_tim_brk_isr_clbk
 */
void reg_tim_brk_isr_clbk(TIM_TypeDef *id, BaseType_t (*cfn)(void *), void *dev);

/**
 * reg_tim_up_isr_clbk
 */
void reg_tim_up_isr_clbk(TIM_TypeDef *id, BaseType_t (*cfn)(void *), void *dev);

/**
 * reg_tim_trg_isr_clbk
 */
void reg_tim_trg_isr_clbk(TIM_TypeDef *id, BaseType_t (*cfn)(void *), void *dev);

/**
 * reg_tim_cc_isr_clbk
 */
void reg_tim_cc_isr_clbk(TIM_TypeDef *id, BaseType_t (*cfn)(void *), void *dev);

#endif
