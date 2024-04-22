/*
 * pwr.c
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
#include "hwerr.h"
#include "tools.h"
#include "clk.h"
#include "fmalloc.h"
#include "flash.h"

static boolean_t wr_opt_byte(volatile uint16_t *adr, uint8_t b);

/**
 * set_flash_wait_states
 */
void set_flash_wait_states(int sysclk)
{
	unsigned int lat;

	if (sysclk <= 24000000) {
		lat = 0;
	} else if (sysclk <= 48000000) {
		lat = 1;
	} else {
		lat = 2;
	}
        FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | lat;
	while (!((FLASH->ACR & FLASH_ACR_LATENCY_Msk) == lat));
}

/**
 * read_flash_opt_bytes
 */
boolean_t read_flash_opt_bytes(struct flash_opt_bytes *ob)
{
	ob->rdp = ((OB->RDP & 0xFF) == 0xA5) ? FALSE : TRUE;
	if (OB->USER & (1 << 2)) {
		ob->rst_stdby = FLASH_USER_RST_STDBY_OFF;
	} else {
		ob->rst_stdby = FLASH_USER_RST_STDBY_ON;
	}
        if (OB->USER & (1 << 1)) {
		ob->rst_stop = FLASH_USER_RST_STOP_OFF;
	} else {
		ob->rst_stop = FLASH_USER_RST_STOP_ON;
	}
        if (OB->USER & (1 << 0)) {
		ob->wdg = FLASH_USER_WDG_SW;
	} else {
		ob->wdg = FLASH_USER_WDG_HW;
	}
	ob->data0 = OB->Data0;
        ob->data1 = OB->Data1;
	ob->wrp = OB->WRP0 & 0xFF;
        ob->wrp |= (OB->WRP1 & 0xFF) << 8;
        ob->wrp |= (OB->WRP2 & 0xFF) << 16;
        ob->wrp |= (OB->WRP3 & 0xFF) << 24;
	if (FLASH->OBR & FLASH_OBR_OPTERR) {
		return (FALSE);
	} else {
		return (TRUE);
	}
}

/**
 * write_flash_opt_bytes
 */
boolean_t write_flash_opt_bytes(struct flash_opt_bytes *ob)
{
	boolean_t ret = TRUE;
	uint8_t u8tmp;

	if (!is_hsi_enabled()) {
		crit_err_exit(UNEXP_PROG_STATE);
	}
	taskENTER_CRITICAL();
        FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
        taskEXIT_CRITICAL();
	if (FLASH->CR & FLASH_CR_LOCK) {
		crit_err_exit(HARDWARE_ERROR);
	}
	taskENTER_CRITICAL();
        FLASH->OPTKEYR = FLASH_OPTKEY1;
	FLASH->OPTKEYR = FLASH_OPTKEY2;
	taskEXIT_CRITICAL();
	if (!(FLASH->CR & FLASH_CR_OPTWRE)) {
		crit_err_exit(HARDWARE_ERROR);
	}
	if (FLASH->SR & FLASH_SR_BSY) {
		crit_err_exit(UNEXP_PROG_STATE);
	}
	// OPTER.
	taskENTER_CRITICAL();
	FLASH->CR |= FLASH_CR_OPTER;
	FLASH->CR |= FLASH_CR_STRT;
	while (FLASH->SR & FLASH_SR_BSY);
	FLASH->CR &= ~FLASH_CR_OPTER;
	taskEXIT_CRITICAL();
	// RDP.
	if (!ob->rdp) {
		if (!wr_opt_byte(&OB->RDP, RDP_KEY)) {
			ret = FALSE;
		}
	} else {
		if (OB->RDP != 0xFFFF) {
			ret = FALSE;
		}
	}
	// USER.
	if (OB->USER != 0xFFFF) {
		ret = FALSE;
	}
	u8tmp = 0xFF;
	if (ob->rst_stdby == FLASH_USER_RST_STDBY_ON) {
		u8tmp &= ~(1 << 2);
	}
	if (ob->rst_stop == FLASH_USER_RST_STOP_ON) {
		u8tmp &= ~(1 << 1);
	}
	if (ob->wdg == FLASH_USER_WDG_HW) {
		u8tmp &= ~(1 << 0);
	}
	if (u8tmp != 0xFF) {
		if (!wr_opt_byte(&OB->USER, u8tmp)) {
			ret = FALSE;
		}
	}
	// Data0.
	if (OB->Data0 != 0xFFFF) {
		ret = FALSE;
	}
	if (ob->data0 != 0xFF) {
		if (!wr_opt_byte(&OB->Data0, ob->data0)) {
			ret = FALSE;
		}
	}
	// Data1.
	if (OB->Data1 != 0xFFFF) {
		ret = FALSE;
	}
	if (ob->data1 != 0xFF) {
		if (!wr_opt_byte(&OB->Data1, ob->data1)) {
			ret = FALSE;
		}
	}
	// WRP0.
        if (OB->WRP0 != 0xFFFF) {
		ret = FALSE;
	}
	u8tmp = ob->wrp;
	if (u8tmp != 0xFF) {
		if (!wr_opt_byte(&OB->WRP0, u8tmp)) {
			ret = FALSE;
		}
	}
	// WRP1.
        if (OB->WRP1 != 0xFFFF) {
		ret = FALSE;
	}
	u8tmp = ob->wrp >> 8;
	if (u8tmp != 0xFF) {
		if (!wr_opt_byte(&OB->WRP1, u8tmp)) {
			ret = FALSE;
		}
	}
	// WRP2.
        if (OB->WRP2 != 0xFFFF) {
		ret = FALSE;
	}
	u8tmp = ob->wrp >> 16;
	if (u8tmp != 0xFF) {
		if (!wr_opt_byte(&OB->WRP2, u8tmp)) {
			ret = FALSE;
		}
	}
	// WRP3.
        if (OB->WRP3 != 0xFFFF) {
		ret = FALSE;
	}
	u8tmp = ob->wrp >> 24;
	if (u8tmp != 0xFF) {
		if (!wr_opt_byte(&OB->WRP3, u8tmp)) {
			ret = FALSE;
		}
	}
	//
	FLASH->CR &= ~FLASH_CR_OPTWRE;
	FLASH->CR |= FLASH_CR_LOCK;
	return (ret);
}

/**
 * wr_opt_byte
 */
static boolean_t wr_opt_byte(volatile uint16_t *adr, uint8_t b)
{
	taskENTER_CRITICAL();
	FLASH->CR |= FLASH_CR_OPTPG;
	*adr = b;
	while (FLASH->SR & FLASH_SR_BSY);
	FLASH->CR &= ~FLASH_CR_OPTPG;
	taskEXIT_CRITICAL();
	if ((*adr & 0xFF) == b) {
		return (TRUE);
	} else {
		return (FALSE);
	}
}

#if TERMOUT == 1
/**
 * log_flash_opt_bytes
 */
void log_flash_opt_bytes(struct flash_opt_bytes *ob)
{
	UBaseType_t prio;
        char *s;

        prio = uxTaskPriorityGet(NULL);
        vTaskPrioritySet(NULL, TASK_PRIO_HIGH);
	msg(INF, "# otion bytes:\n");
	msg(INF, "rst_stdby: %s\n", (ob->rst_stdby == FLASH_USER_RST_STDBY_ON) ? "ON" : "OFF");
	msg(INF, "rst_stop: %s\n", (ob->rst_stop == FLASH_USER_RST_STOP_ON) ? "ON" : "OFF");
        msg(INF, "wdg: %s\n", (ob->wdg == FLASH_USER_WDG_HW) ? "HW" : "SW");
        msg(INF, "rdp: %s\n", (ob->rdp) ? "ON" : "OFF");
        if (NULL == (s = pvPortMalloc(64))) {
		crit_err_exit(MALLOC_ERROR);
	}
	prn_bv_str(s, ob->wrp, 32);
	msg(INF, "wrp: %s\n", s);
	prn_bv_str(s, ob->data0, 8);
	msg(INF, "data0: %s\n", s);
	prn_bv_str(s, ob->data1, 8);
	msg(INF, "data1: %s\n", s);
        vPortFree(s);
        msg(INF, "#\n");
	vTaskPrioritySet(NULL, prio);
}
#endif
