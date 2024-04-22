/*
 * rst.c
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
#include "rst.h"

#if TERMOUT == 1
static const char *const reset_cause_ary[] = {"LOWPWR", "WINWDT", "INDWDT", "SOFT", "PORPDR", "EXTPIN", "ERR"};

static void tsk(void *p);
#endif

/**
 * reset_peripheral
 */
void reset_peripheral(enum rst_apb_inst_sel apb, unsigned int bmp)
{
	switch (apb) {
	case RST_LOW_SPEED_APB1  :
		taskENTER_CRITICAL();
                RCC->APB1RSTR |= bmp;
		while (!(RCC->APB1RSTR & bmp));
                RCC->APB1RSTR &= ~bmp;
		taskEXIT_CRITICAL();
		break;
	case RST_HIGH_SPEED_APB2 :
		taskENTER_CRITICAL();
                RCC->APB2RSTR |= bmp;
		while (!(RCC->APB2RSTR & bmp));
                RCC->APB2RSTR &= ~bmp;
		taskEXIT_CRITICAL();
		break;
	default                  :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
}

/**
 * reset_backup_domain
 */
void reset_backup_domain(void)
{
	RCC->BDCR |= RCC_BDCR_BDRST;
	while (!(RCC->BDCR & RCC_BDCR_BDRST));
	RCC->BDCR &= ~RCC_BDCR_BDRST;
}

/**
 * get_reset_cause
 */
enum reset_cause get_reset_cause(void)
{
	enum reset_cause ret = RESET_CAUSE_ERR;

	if (RCC->CSR & RCC_CSR_PORRSTF) {
		ret = RESET_CAUSE_POR_PDR;
	} else if (RCC->CSR & RCC_CSR_SFTRSTF) {
		ret = RESET_CAUSE_SOFT;
	} else if (RCC->CSR & RCC_CSR_IWDGRSTF) {
		ret = RESET_CAUSE_IND_WDT;
        } else if (RCC->CSR & RCC_CSR_WWDGRSTF) {
		ret = RESET_CAUSE_WIN_WDT;
        } else if (RCC->CSR & RCC_CSR_LPWRRSTF) {
		ret = RESET_CAUSE_LOW_PWR;
	} else if (RCC->CSR & RCC_CSR_PINRSTF) { // PINRSTF flag is always set.
		ret = RESET_CAUSE_NRST;
	}
	RCC->CSR |= RCC_CSR_RMVF;
	return (ret);
}

#if TERMOUT == 1
/**
 * get_reset_cause_str
 */
const char *get_reset_cause_str(enum reset_cause cause)
{
	return (reset_cause_ary[cause]);
}
#endif

/**
 * reset_request
 */
void reset_request(char *msg)
{
#if TERMOUT == 1
        if (pdPASS != xTaskCreate(tsk, "RESET", RESET_TASK_STACK_SIZE, msg,
				  TASK_PRIO_HIGH, NULL)) {
                crit_err_exit(MALLOC_ERROR);
        }
#else
        NVIC_SystemReset();
#endif
}

#if TERMOUT == 1
/**
 * tsk
 */
static void tsk(void *p)
{
	char *dummy;

	msg(INF, ">>>>>>>>>>> System Reset (%s)\n", (char *) p);
	disable_tout();
	vTaskPrioritySet(tout_tsk_hndl(), TASK_PRIO_HIGH);
	while (pdTRUE == xQueuePeek(tout_mque(), &dummy, 0)) {
		taskYIELD();
	}
	vTaskDelay(100 / portTICK_PERIOD_MS);
	NVIC_SystemReset();
}
#endif
