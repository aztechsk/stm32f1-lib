/*
 * devid.c
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
#include "devid.h"

struct devid devid;

/**
 * init_devid
 */
void init_devid(void)
{
	uint32_t tmp;
	int sz, of;

	devid.flash_sz = *((uint16_t *) FLASHSIZE_BASE);
        tmp = *((uint32_t *) (UID_BASE + 8));
	sz = 4;
	of = 0;
	for (int i = sz - 1; i >= 0; i--) {
		devid.serial[sz - 1 - i + of] = tmp >> i * 8;
	}
        tmp = *((uint32_t *) (UID_BASE + 4));
	sz = 4;
	of = 4;
	for (int i = sz - 1; i >= 0; i--) {
		devid.serial[sz - 1 - i + of] = tmp >> i * 8;
	}
        tmp = *((uint32_t *) (UID_BASE + 2));
	sz = 2;
	of = 8;
	for (int i = sz - 1; i >= 0; i--) {
		devid.serial[sz - 1 - i + of] = tmp >> i * 8;
	}
        tmp = *((uint32_t *) UID_BASE);
	sz = 2;
	of = 10;
	for (int i = sz - 1; i >= 0; i--) {
		devid.serial[sz - 1 - i + of] = tmp >> i * 8;
	}
        devid.dev_id = DBGMCU->IDCODE & DBGMCU_IDCODE_DEV_ID;
	devid.rev_id = DBGMCU->IDCODE >> DBGMCU_IDCODE_REV_ID_Pos;
}

#if TERMOUT == 1
/**
 * log_dev_id
 */
void log_dev_id(void)
{
	const char *dev_id, *rev_id;

	switch (devid.dev_id) {
	case 0x412 :
		dev_id = "low";
		if (devid.rev_id == 0x1000) {
			rev_id = "A";
		} else {
			rev_id = "rev_id_err";
		}
		break;
	case 0x410 :
		dev_id = "medium";
		switch (devid.rev_id) {
		case 0x0000 :
			rev_id = "A";
			break;
		case 0x2000 :
			rev_id = "B";
			break;
		case 0x2001 :
			rev_id = "Z";
			break;
		case 0x2003 :
			rev_id = "1, 2, 3, X, Y";
                	break;
		default :
			rev_id = "rev_id_err";
			break;
		}
		break;
	case 0x414 :
		dev_id = "high";
		switch (devid.rev_id) {
		case 0x1000 :
			rev_id = "A, 1";
			break;
		case 0x1001 :
			rev_id = "Z";
			break;
		case 0x1003 :
			rev_id = "1, 2, 3, X, Y";
			break;
		default :
			rev_id = "rev_id_err";
			break;
		}
		break;
	case 0x430 :
		dev_id = "xl";
		if (devid.rev_id == 0x1000) {
			rev_id = "A, 1";
		} else {
			rev_id = "rev_id_err";
		}
		break;
	case 0x418 :
		dev_id = "connect";
		switch (devid.rev_id) {
		case 0x1000 :
			rev_id = "A";
			break;
		case 0x1001 :
			rev_id = "Z";
			break;
		default :
			rev_id = "rev_id_err";
			break;
		}
		break;
	default :
		dev_id = "dev_id_err";
                rev_id = "rev_id_err";
		break;
	}
	msg(INF, "devid.c: STM32F1 %s-density, rev: [%s], flash %d kB\n", dev_id, rev_id, devid.flash_sz);
}

/**
 * log_dev_sn
 */
void log_dev_sn(void)
{
	msg(INF, "devid.c: sn=");
	for (int i = 0; i < DEVID_SN_SZ; i++) {
		msg(INF, "%02hX", devid.serial[i]);
	}
	msg(INF, "\n");
}
#endif
