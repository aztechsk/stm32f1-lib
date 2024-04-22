/*
 * btn.c
 *
 * Copyright (c) 2021 Jan Rusnak <jan@rusnak.sk>
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
#include "extint.h"
#if BTN_SLEEP == 1
#include "sleep.h"
#endif
#include "btn.h"

#if BTN == 1

struct isr_msg {
	TickType_t tm;
	boolean_t intr_sig;
};

static btn btn_list;
static int isr_clbk_num;

static void btn_tsk(void *p);
static BaseType_t isr_clbk(unsigned int pend);
#if BTN_SLEEP == 1
static void sleep_clbk(enum sleep_cmd cmd, ...);
#endif

/**
 * add_btn_dev
 */
void add_btn_dev(btn dev)
{
	dev->intr_que = xQueueCreate(1, sizeof(struct isr_msg));
	if (dev->intr_que == NULL) {
		crit_err_exit(MALLOC_ERROR);
	}
	dev->evnt_que = xQueueCreate(dev->evnt_que_size, sizeof(struct btn_evnt));
	if (dev->evnt_que == NULL) {
		crit_err_exit(MALLOC_ERROR);
	}
	if (!is_extint_intr_init(dev->pin)) {
		crit_err_exit(UNEXP_PROG_STATE);
	}
        reg_extint_isr_clbk(dev->pin, isr_clbk);
        taskENTER_CRITICAL();
	if (btn_list) {
		btn b = btn_list;
		while (b->next) {
			b = b->next;
		}
		b->next = dev;
	} else {
		btn_list = dev;
	}
	taskEXIT_CRITICAL();
	if (pdPASS != xTaskCreate(btn_tsk, dev->tsk_nm, BTN_TASK_STACK_SIZE, dev, BTN_TASK_PRIO, &dev->tsk_hndl)) {
                crit_err_exit(MALLOC_ERROR);
        }
        map_extint_line(dev->pin, dev->port);
	if (dev->act_lev) {
		conf_gpio(dev->pin, dev->port, GPIO_FUNC_INPUT, GPIO_FEAT_PULL_DOWN, GPIO_FEAT_END);
                set_extint_line_mode(dev->pin, EXTINT_LINE_RISE_TRIG);
	} else {
		conf_gpio(dev->pin, dev->port, GPIO_FUNC_INPUT, GPIO_FEAT_PULL_UP, GPIO_FEAT_END);
                set_extint_line_mode(dev->pin, EXTINT_LINE_FALL_TRIG);
	}
#if BTN_SLEEP == 1
	reg_sleep_clbk(sleep_clbk, SLEEP_PRIO_SUSP_FIRST);
#endif
}

/**
 * enable_btn_intr
 */
void enable_btn_intr(btn dev)
{
	enable_extint_intr(dev->pin);
}

/**
 * btn_tsk
 */
static void btn_tsk(void *p)
{
	btn b = p;
        struct btn_evnt evnt;
        struct isr_msg msg;
	int cnt;
	boolean_t qfull = FALSE;

        vTaskDelay(20 / portTICK_PERIOD_MS);
        while (TRUE) {
                enable_extint_intr(b->pin);
		xQueueReceive(b->intr_que, &msg, portMAX_DELAY);
#if BTN_SLEEP == 1
		if (msg.intr_sig) {
                        msg(INF, "btn.c: (%s) suspend\n", b->tsk_nm);
                        vTaskSuspend(NULL);
			if (pdTRUE == xQueueReceive(b->intr_que, &msg, 0)) {
				cnt = 0;
				while (TRUE) {
					vTaskDelay(BTN_CHECK_DELAY / portTICK_PERIOD_MS);
					if (b->act_lev) {
						if (!get_gpio_lev(b->pin, b->port)) {
							cnt++;
						} else {
							cnt = 0;
						}
					} else {
						if (get_gpio_lev(b->pin, b->port)) {
							cnt++;
						} else {
							cnt = 0;
						}
					}
					if (cnt == BTN_CHECK_DELAY_CNT) {
						break;
					}
				}
			}
                        continue;
		}
#endif
		if (b->mode == BTN_EVENT_MODE) {
			qfull = FALSE;
			evnt.type = BTN_PRESS;
                        evnt.time = msg.tm;
			if (errQUEUE_FULL == xQueueSend(b->evnt_que, &evnt, 0)) {
                                qfull = TRUE;
				b->evnt_que_full++;
			}
		}
                cnt = 0;
                while (TRUE) {
			vTaskDelay(BTN_CHECK_DELAY / portTICK_PERIOD_MS);
			if (b->act_lev) {
				if (!get_gpio_lev(b->pin, b->port)) {
					cnt++;
				} else {
					cnt = 0;
				}
			} else {
				if (get_gpio_lev(b->pin, b->port)) {
					cnt++;
				} else {
					cnt = 0;
				}
			}
			if (cnt == BTN_CHECK_DELAY_CNT) {
				if (b->mode == BTN_EVENT_MODE) {
					if (qfull) {
						break;
					}
					evnt.type = BTN_RELEASE;
					evnt.time = xTaskGetTickCount();
				} else {
					evnt.type = BTN_PRESSED_DOWN;
                                        evnt.time = xTaskGetTickCount() - msg.tm;

				}
				xQueueSend(b->evnt_que, &evnt, portMAX_DELAY);
				break;
			}
		}
	}
}

/**
 * isr_clbk
 */
static BaseType_t isr_clbk(unsigned int pend)
{
	BaseType_t tsk_wkn = pdFALSE;
        struct isr_msg msg;
	btn b = btn_list;

	isr_clbk_num++;
        while (b != NULL) {
		if (pend & b->pin) {
			b->clbk_num++;
			if (b->act_lev) {
				if (!get_gpio_lev(b->pin, b->port)) {
					clear_extint_intr(b->pin);
					b->pin_lev_err++;
					b = b->next;
					continue;
				}
			} else {
				if (get_gpio_lev(b->pin, b->port)) {
                                        clear_extint_intr(b->pin);
					b->pin_lev_err++;
					b = b->next;
					continue;
				}
			}
                        disable_extint_intr_uns(b->pin);
                        clear_extint_intr(b->pin);
			msg.tm = xTaskGetTickCountFromISR();
			msg.intr_sig = FALSE;
			BaseType_t wkn = pdFALSE;
			if (errQUEUE_FULL == xQueueSendFromISR(b->intr_que, &msg, &wkn)) {
				b->intr_que_full++;
			}
			if (wkn) {
				tsk_wkn = pdTRUE;
			}
		}
                b = b->next;
	}
        return (tsk_wkn);
}

#if BTN_SLEEP == 1
/**
 * sleep_clbk
 */
static void sleep_clbk(enum sleep_cmd cmd, ...)
{
	btn b;

	if (btn_list) {
		b = btn_list;
	} else {
		return;
	}
	if (cmd == SLEEP_CMD_SUSP) {
		do {
                        taskENTER_CRITICAL();
			disable_extint_intr_uns(b->pin);
                        clear_extint_intr(b->pin);
                        taskEXIT_CRITICAL();
			struct isr_msg msg;
                        msg.tm = 0;
			msg.intr_sig = TRUE;
			xQueueSend(b->intr_que, &msg, portMAX_DELAY);
			while (eSuspended != eTaskGetState(b->tsk_hndl)) {
				taskYIELD();
			}
		} while ((b = b->next));
	} else {
		do {
			vTaskResume(b->tsk_hndl);
		} while ((b = b->next));
	}
}
#endif

#if TERMOUT == 1
/**
 * log_btn_stats
 */
void log_btn_stats(btn dev)
{
	msg(INF, "btn.c: <%s> intr_que_full=%d evnt_que_full=%d\n",
	    dev->tsk_nm, dev->intr_que_full, dev->evnt_que_full);
	msg(INF, "btn.c: <%s> pin_lev_err=%d clbk_num=%d\n",
	    dev->tsk_nm, dev->pin_lev_err, dev->clbk_num);
	msg(INF, "btn.c: clbk_num_all=%d\n", isr_clbk_num);

}
#endif

#endif
