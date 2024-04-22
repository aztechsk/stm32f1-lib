/*
 * sigidle.c
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
#include "msgconf.h"
#include "criterr.h"
#include "extint.h"
#include "sigidle.h"

#if SIGIDLE == 1

static sigidle si_list;

static BaseType_t extint_clbk(unsigned int pend);
static BaseType_t tim_clbk(void *dev);

/**
 * init_sigidle
 */
void init_sigidle(sigidle dev)
{
	if (!is_extint_intr_init(dev->pin)) {
		crit_err_exit(UNEXP_PROG_STATE);
	}
        taskENTER_CRITICAL();
	if (si_list) {
		sigidle si = si_list;
		while (si->next) {
			si = si->next;
		}
		si->next = dev;
	} else {
		si_list = dev;
	}
	taskEXIT_CRITICAL();
        reg_extint_isr_clbk(dev->pin, extint_clbk);
        map_extint_line(dev->pin, dev->port);
        set_extint_line_mode(dev->pin, EXTINT_LINE_EDGE_TRIG);
	dev->tim->mmio = dev->tim_mmio;
	dev->tim->isr_clbk = tim_clbk;
        dev->tim->tm_us = dev->tm_us;
        dev->tim->owner = dev;
	init_timer(dev->tim, TIMER_PERIODIC_ISR_MODE);
}

/**
 * sigidle_wait
 */
void sigidle_wait(sigidle dev)
{
	if (dev->tm_us != dev->tim->tm_us) {
		dev->tim->tm_us = dev->tm_us;
		reconf_timer(dev->tim);
	}
        dev->c_tsk = xTaskGetCurrentTaskHandle();
	taskENTER_CRITICAL();
	enable_extint_intr_uns(dev->pin);
        start_timer(dev->tim);
	taskEXIT_CRITICAL();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

/**
 * extint_clbk
 */
static BaseType_t extint_clbk(unsigned int pend)
{
	sigidle si = si_list;

        while (si != NULL) {
		if (pend & si->pin) {
			clear_extint_intr(si->pin);
                        restart_timer(si->tim);
		}
                si = si->next;
	}
        return (pdFALSE);
}

/**
 * tim_clbk
 */
static BaseType_t tim_clbk(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	sigidle si = ((timer) dev)->owner;

	if (si->idle_lev == get_gpio_lev(si->pin, si->port)) {
		disable_extint_intr_uns(si->pin);
		clear_extint_intr(si->pin);
                stop_timer(dev);
		vTaskNotifyGiveFromISR(si->c_tsk, &tsk_wkn);
	} else {
		clear_timer_sr(dev);
	}
        return (tsk_wkn);
}

#endif
