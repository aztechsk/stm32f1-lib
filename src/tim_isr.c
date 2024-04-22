/*
 * tim_isr.c
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
#include "tim_isr.h"

struct clbk {
	void *dev;
        BaseType_t (*cfn)(void *);
};

struct adv_clbk {
	void *dev;
        BaseType_t (*cfn_brk)(void *);
        BaseType_t (*cfn_up)(void *);
        BaseType_t (*cfn_trg)(void *);
        BaseType_t (*cfn_cc)(void *);
};

#ifdef TIM2
static struct clbk clbk2;

/**
 * TIM2_IRQHandler
 */
void TIM2_IRQHandler(void);
void TIM2_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk2.cfn)(clbk2.dev));
}
#endif

#ifdef TIM3
static struct clbk clbk3;

/**
 * TIM3_IRQHandler
 */
void TIM3_IRQHandler(void);
void TIM3_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk3.cfn)(clbk3.dev));
}
#endif

#ifdef TIM4
static struct clbk clbk4;

/**
 * TIM4_IRQHandler
 */
void TIM4_IRQHandler(void);
void TIM4_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk4.cfn)(clbk4.dev));
}
#endif

#ifdef TIM5
static struct clbk clbk5;

/**
 * TIM5_IRQHandler
 */
void TIM5_IRQHandler(void);
void TIM5_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk5.cfn)(clbk5.dev));
}
#endif

#ifdef TIM1
static struct adv_clbk adv_clbk1;

/**
 * TIM1_BRK_IRQHandler
 */
void TIM1_BRK_IRQHandler(void);
void TIM1_BRK_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*adv_clbk1.cfn_brk)(adv_clbk1.dev));
}

/**
 * TIM1_UP_IRQHandler
 */
void TIM1_UP_IRQHandler(void);
void TIM1_UP_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*adv_clbk1.cfn_up)(adv_clbk1.dev));
}

/**
 * TIM1_TRG_COM_IRQHandler
 */
void TIM1_TRG_COM_IRQHandler(void);
void TIM1_TRG_COM_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*adv_clbk1.cfn_trg)(adv_clbk1.dev));
}

/**
 * TIM1_CC_IRQHandler
 */
void TIM1_CC_IRQHandler(void);
void TIM1_CC_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*adv_clbk1.cfn_cc)(adv_clbk1.dev));
}
#endif

#ifdef TIM8
static struct adv_clbk adv_clbk8;

/**
 * TIM8_BRK_IRQHandler
 */
void TIM8_BRK_IRQHandler(void);
void TIM8_BRK_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*adv_clbk8.cfn_brk)(adv_clbk8.dev));
}

/**
 * TIM8_UP_IRQHandler
 */
void TIM8_UP_IRQHandler(void);
void TIM8_UP_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*adv_clbk8.cfn_up)(adv_clbk8.dev));
}

/**
 * TIM8_TRG_COM_IRQHandler
 */
void TIM8_TRG_COM_IRQHandler(void);
void TIM8_TRG_COM_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*adv_clbk8.cfn_trg)(adv_clbk8.dev));
}

/**
 * TIM8_CC_IRQHandler
 */
void TIM8_CC_IRQHandler(void);
void TIM8_CC_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*adv_clbk8.cfn_cc)(adv_clbk8.dev));
}
#endif

/**
 * reg_tim_isr_clbk
 */
void reg_tim_isr_clbk(TIM_TypeDef *id, BaseType_t (*cfn)(void *), void *dev)
{
#ifdef TIM2
	if (id == TIM2) {
		clbk2.cfn = cfn;
		clbk2.dev = dev;
		return;
	}
#endif
#ifdef TIM3
	if (id == TIM3) {
		clbk3.cfn = cfn;
		clbk3.dev = dev;
		return;
	}
#endif
#ifdef TIM4
	if (id == TIM4) {
		clbk4.cfn = cfn;
		clbk4.dev = dev;
		return;
	}
#endif
#ifdef TIM5
	if (id == TIM5) {
		clbk5.cfn = cfn;
		clbk5.dev = dev;
		return;
	}
#endif
	crit_err_exit(BAD_PARAMETER);
}

/**
 * reg_tim_brk_isr_clbk
 */
void reg_tim_brk_isr_clbk(TIM_TypeDef *id, BaseType_t (*cfn)(void *), void *dev)
{
#ifdef TIM1
	if (id == TIM1) {
		adv_clbk1.cfn_brk = cfn;
		adv_clbk1.dev = dev;
		return;
	}
#endif
#ifdef TIM8
	if (id == TIM8) {
		adv_clbk8.cfn_brk = cfn;
		adv_clbk8.dev = dev;
		return;
	}
#endif
	crit_err_exit(BAD_PARAMETER);
}

/**
 * reg_tim_up_isr_clbk
 */
void reg_tim_up_isr_clbk(TIM_TypeDef *id, BaseType_t (*cfn)(void *), void *dev)
{
#ifdef TIM1
	if (id == TIM1) {
		adv_clbk1.cfn_up = cfn;
		adv_clbk1.dev = dev;
		return;
	}
#endif
#ifdef TIM8
	if (id == TIM8) {
		adv_clbk8.cfn_up = cfn;
		adv_clbk8.dev = dev;
		return;
	}
#endif
	crit_err_exit(BAD_PARAMETER);
}

/**
 * reg_tim_trg_isr_clbk
 */
void reg_tim_trg_isr_clbk(TIM_TypeDef *id, BaseType_t (*cfn)(void *), void *dev)
{
#ifdef TIM1
	if (id == TIM1) {
		adv_clbk1.cfn_trg = cfn;
		adv_clbk1.dev = dev;
		return;
	}
#endif
#ifdef TIM8
	if (id == TIM8) {
		adv_clbk8.cfn_trg = cfn;
		adv_clbk8.dev = dev;
		return;
	}
#endif
	crit_err_exit(BAD_PARAMETER);
}

/**
 * reg_tim_cc_isr_clbk
 */
void reg_tim_cc_isr_clbk(TIM_TypeDef *id, BaseType_t (*cfn)(void *), void *dev)
{
#ifdef TIM1
	if (id == TIM1) {
		adv_clbk1.cfn_cc = cfn;
		adv_clbk1.dev = dev;
		return;
	}
#endif
#ifdef TIM8
	if (id == TIM8) {
		adv_clbk8.cfn_cc = cfn;
		adv_clbk8.dev = dev;
		return;
	}
#endif
	crit_err_exit(BAD_PARAMETER);
}
