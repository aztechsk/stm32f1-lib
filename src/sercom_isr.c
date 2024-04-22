/*
 * sercom_isr.c
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
#include "sercom_isr.h"

struct clbk {
	void *dev;
        BaseType_t (*cfn)(void *);
};

#ifdef USART1
static struct clbk clbk1;

/**
 * USART1_IRQHandler
 */
void USART1_IRQHandler(void);
void USART1_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk1.cfn)(clbk1.dev));
}
#endif

#ifdef USART2
static struct clbk clbk2;

/**
 * USART2_IRQHandler
 */
void USART2_IRQHandler(void);
void USART2_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk2.cfn)(clbk2.dev));
}
#endif

#ifdef USART3
static struct clbk clbk3;

/**
 * USART3_IRQHandler
 */
void USART3_IRQHandler(void);
void USART3_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk3.cfn)(clbk3.dev));
}
#endif

/**
 * reg_sercom_isr_clbk
 */
void reg_sercom_isr_clbk(IRQn_Type id, BaseType_t (*cfn)(void *), void *dev)
{
#ifdef USART1
	if (id == USART1_IRQn) {
		clbk1.cfn = cfn;
		clbk1.dev = dev;
		return;
	}
#endif
#ifdef USART2
	if (id == USART2_IRQn) {
		clbk2.cfn = cfn;
		clbk2.dev = dev;
		return;
	}
#endif
#ifdef USART3
	if (id == USART3_IRQn) {
		clbk3.cfn = cfn;
		clbk3.dev = dev;
		return;
	}
#endif
	crit_err_exit(BAD_PARAMETER);
}
