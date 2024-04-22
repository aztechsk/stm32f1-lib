/*
 * i2c_isr.c
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
#include "i2c_isr.h"

#if I2CM == 1

struct clbk {
	void *dev;
        BaseType_t (*cfn_ev)(void *);
	BaseType_t (*cfn_er)(void *);
};

#ifdef I2C1
static struct clbk clbk1;

/**
 * I2C1_EV_IRQHandler
 */
void I2C1_EV_IRQHandler(void);
void I2C1_EV_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk1.cfn_ev)(clbk1.dev));
}

/**
 * I2C1_ER_IRQHandler
 */
void I2C1_ER_IRQHandler(void);
void I2C1_ER_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk1.cfn_er)(clbk1.dev));
}
#endif

#ifdef I2C2
static struct clbk clbk2;

/**
 * I2C2_EV_IRQHandler
 */
void I2C2_EV_IRQHandler(void);
void I2C2_EV_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk2.cfn_ev)(clbk2.dev));
}

/**
 * I2C2_ER_IRQHandler
 */
void I2C2_ER_IRQHandler(void);
void I2C2_ER_IRQHandler(void)
{
	portEND_SWITCHING_ISR((*clbk2.cfn_er)(clbk2.dev));
}
#endif

/**
 * reg_i2c_isr_clbks
 */
void reg_i2c_isr_clbks(I2C_TypeDef *id, BaseType_t (*cfn_ev)(void *), BaseType_t (*cfn_er)(void *),
                       void *dev)
{
#ifdef I2C1
	if (id == I2C1) {
		clbk1.cfn_ev = cfn_ev;
		clbk1.cfn_er = cfn_er;
		clbk1.dev = dev;
		return;
	}
#endif
#ifdef I2C2
	if (id == I2C2) {
		clbk2.cfn_ev = cfn_ev;
		clbk2.cfn_er = cfn_er;
		clbk2.dev = dev;
		return;
	}
#endif
	crit_err_exit(BAD_PARAMETER);
}

/**
 * reg_i2c_ev_isr_clbk
 */
void reg_i2c_ev_isr_clbk(I2C_TypeDef *id, BaseType_t (*cfn)(void *))
{
#ifdef I2C1
	if (id == I2C1) {
		clbk1.cfn_ev = cfn;
		return;
	}
#endif
#ifdef I2C2
	if (id == I2C2) {
		clbk2.cfn_ev = cfn;
		return;
	}
#endif
}

/**
 * get_i2c_ev_isr_clbk
 */
BaseType_t (*get_i2c_ev_isr_clbk(I2C_TypeDef *id))(void *)
{
#ifdef I2C1
	if (id == I2C1) {
		return (clbk1.cfn_ev);
	}
#endif
#ifdef I2C2
	if (id == I2C2) {
		return (clbk2.cfn_ev);
	}
#endif
	crit_err_exit(BAD_PARAMETER);
	return (NULL);
}

#endif
