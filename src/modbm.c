/*
 * modbm.c
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
#include "fmalloc.h"
#include "hwerr.h"
#include "clk.h"
#include "crc.h"
#include "sercom_isr.h"
#include "modbm.h"

#if MODBM == 1

enum fn_codes {
	RD_COILS = 0x01,
	RD_DISCRETE_INPUTS = 0x02,
        RD_HOLDING_REGISTERS = 0x03,
        RD_INPUT_REGISTERS = 0x04,
        WR_SINGLE_COIL = 0x05,
        WR_SINGLE_REGISTER = 0x06,
        WR_MULTIPLE_COILS = 0x0F,
        WR_MULTIPLE_REGISTERS = 0x10
};

static int wait_trans(modbm dev, int adr, int tmo);
static void dis_rx(modbm dev);
static BaseType_t tx_hndlr(void *dev);
static BaseType_t rx_hndlr(void *dev);
static BaseType_t idle_hndlr(void *dev);
static unsigned int reg_brr_val(modbm dev);

/**
 * init_modbm
 */
void init_modbm(modbm dev)
{
#ifdef USART1
	if (dev->instance == MODBM_USART1) {
		dev->irqn = USART1_IRQn;
                dev->apb_inst = CLK_HIGH_SPEED_APB2;
                dev->apb_bmp =  RCC_APB2ENR_USART1EN;
                dev->mmio = USART1;
		goto lb1;
	}
#endif
#ifdef USART2
	if (dev->instance == MODBM_USART2) {
		dev->irqn = USART2_IRQn;
                dev->apb_inst = CLK_LOW_SPEED_APB1;
                dev->apb_bmp = RCC_APB1ENR_USART2EN;
                dev->mmio = USART2;
		goto lb1;
	}
#endif
#ifdef USART3
	if (dev->instance == MODBM_USART3) {
		dev->irqn = USART3_IRQn;
                dev->apb_inst = CLK_LOW_SPEED_APB1;
                dev->apb_bmp = RCC_APB1ENR_USART3EN;
                dev->mmio = USART3;
		goto lb1;
	}
#endif
	crit_err_exit(BAD_PARAMETER);
lb1:
	if (dev->sig_que == NULL) {
		if (NULL == (dev->sig_que = xQueueCreate(1, sizeof(uint8_t)))) {
			crit_err_exit(MALLOC_ERROR);
		}
	} else {
		crit_err_exit(UNEXP_PROG_STATE);
	}
	if (NULL == (dev->buf = pvPortMalloc(dev->buf_len))) {
		crit_err_exit(MALLOC_ERROR);
	}
	dev->reg_brr = reg_brr_val(dev);
	dev->reg_cr1 = 0;
	if (dev->parity != MODBM_PARITY_NO) {
		dev->reg_cr1 |= USART_CR1_PCE;
		if (dev->parity == MODBM_PARITY_ODD) {
			dev->reg_cr1 |= USART_CR1_PS;
		}
                dev->reg_cr1 |= USART_CR1_M;
	}
        dev->reg_cr2 = 0;
	if (dev->parity == MODBM_PARITY_NO) {
		dev->reg_cr2 |= 2 << USART_CR2_STOP_Pos;
	}
        dev->reg_cr3 = 0;
        NVIC_DisableIRQ(dev->irqn);
        enable_per_apb_clk(dev->apb_inst, dev->apb_bmp);
	dev->mmio->CR1 = dev->reg_cr1 | USART_CR1_UE;
	while (!(dev->mmio->CR1 & USART_CR1_UE));
	dev->mmio->CR2 = dev->reg_cr2;
	dev->mmio->CR3 = dev->reg_cr3;
	dev->mmio->BRR = dev->reg_brr;
	dev->mmio->SR = 0;
        dev->mmio->CR1 |= USART_CR1_TE;
        dev->conf_pins(MODBM_PINS_PERIPHERAL);
        NVIC_SetPriority(dev->irqn, configLIBRARY_MAX_API_CALL_INTERRUPT_PRIORITY);
        NVIC_ClearPendingIRQ(dev->irqn);
	NVIC_EnableIRQ(dev->irqn);
}

/**
 * modbm_read_registers
 */
int modbm_read_registers(modbm dev, int adr, enum modbm_reg_table table, struct modbm_reg_io_req *req)
{
 	*dev->buf = adr;
	if (table == MODBM_INPUT_REG_TABLE) {
		*(dev->buf + 1) = RD_INPUT_REGISTERS;
	} else if (table == MODBM_HOLDING_REG_TABLE) {
		*(dev->buf + 1) = RD_HOLDING_REGISTERS;
	} else {
		crit_err_exit(BAD_PARAMETER);
	}
	if (req->reg_cnt < 1 || req->reg_cnt > 125) {
		crit_err_exit(BAD_PARAMETER);
	}
	int radr = req->reg_start - 1;
	if (radr < 0 || radr + req->reg_cnt - 1 > 0xFFFF) {
		crit_err_exit(BAD_PARAMETER);
	}
	*(dev->buf + 2) = radr >> 8;
        *(dev->buf + 3) = radr;
	*(dev->buf + 4) = req->reg_cnt >> 8;
        *(dev->buf + 5) = req->reg_cnt;
	uint16_t crc = crc_16(INIT_CRC_16_MODB, dev->buf, 6);
	*(dev->buf + 6) = crc;
        *(dev->buf + 7) = crc >> 8;
	dev->s_sz = 8;
	dev->r_cnt = dev->r_sz = req->reg_cnt * 2 + 5;
	if (dev->r_sz > dev->buf_len) {
		crit_err_exit(BAD_PARAMETER);
	}
	int ret;
	if (0 != (ret = wait_trans(dev, adr, req->tmo_ms))) {
		return (ret);
	}
	int fn = (table == MODBM_INPUT_REG_TABLE) ? RD_INPUT_REGISTERS : RD_HOLDING_REGISTERS;
	if (*(dev->buf + 1) == fn) {
		req->except = 0;
	} else if (*(dev->buf + 1) == (fn | 0x80)) {
		req->except = *(dev->buf + 2);
		return (-ENACK);
	} else {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	if (req->reg_cnt * 2 != *(dev->buf + 2)) {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	for (int i = 0; i < req->reg_cnt; i++) {
		req->regs[i] = *(dev->buf + 3 + i * 2) << 8;
		req->regs[i] |= *(dev->buf + 3 + i * 2 + 1);
	}
	return (0);
}

/**
 * modbm_write_registers
 */
int modbm_write_registers(modbm dev, int adr, struct modbm_reg_io_req *req)
{
	*dev->buf = adr;
        *(dev->buf + 1) = WR_MULTIPLE_REGISTERS;
	if (req->reg_cnt < 1 || req->reg_cnt > 123) {
		crit_err_exit(BAD_PARAMETER);
	}
	int radr = req->reg_start - 1;
	if (radr < 0 || radr + req->reg_cnt - 1 > 0xFFFF) {
		crit_err_exit(BAD_PARAMETER);
	}
	*(dev->buf + 2) = radr >> 8;
        *(dev->buf + 3) = radr;
	*(dev->buf + 4) = req->reg_cnt >> 8;
        *(dev->buf + 5) = req->reg_cnt;
	*(dev->buf + 6) = dev->s_sz = 2 * req->reg_cnt;
        dev->s_sz += 9;
	if (dev->s_sz > dev->buf_len) {
		crit_err_exit(BAD_PARAMETER);
	}
	for (int i = 0; i < req->reg_cnt; i++) {
		*(dev->buf + 7 + i * 2) = req->regs[i] >> 8;
                *(dev->buf + 7 + i * 2 + 1) = req->regs[i];
	}
	uint16_t crc = crc_16(INIT_CRC_16_MODB, dev->buf, dev->s_sz - 2);
	*(dev->buf + 7 + 2 * req->reg_cnt) = crc;
        *(dev->buf + 7 + 2 * req->reg_cnt + 1) = crc >> 8;
        dev->r_cnt = dev->r_sz = 8;
	int ret;
	if (0 != (ret = wait_trans(dev, adr, req->tmo_ms))) {
		return (ret);
	}
	if (*(dev->buf + 1) == WR_MULTIPLE_REGISTERS) {
		req->except = 0;
	} else if (*(dev->buf + 1) == (WR_MULTIPLE_REGISTERS | 0x80)) {
		req->except = *(dev->buf + 2);
		return (-ENACK);
	} else {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	int tmp = *(dev->buf + 2) << 8;
	tmp |= *(dev->buf + 3);
	if (tmp != radr) {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	tmp = *(dev->buf + 4) << 8;
	tmp |= *(dev->buf + 5);
	if (tmp != req->reg_cnt) {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	return (0);
}

/**
 * modbm_write_register
 */
int modbm_write_register(modbm dev, int adr, struct modbm_reg_io_req *req)
{
	*dev->buf = adr;
        *(dev->buf + 1) = WR_SINGLE_REGISTER;
	int radr = req->reg_start - 1;
	if (radr < 0 || radr > 0xFFFF) {
		crit_err_exit(BAD_PARAMETER);
	}
	*(dev->buf + 2) = radr >> 8;
        *(dev->buf + 3) = radr;
	*(dev->buf + 4) = req->regs[0] >> 8;
        *(dev->buf + 5) = req->regs[0];
        dev->s_sz = 8;
	uint16_t crc = crc_16(INIT_CRC_16_MODB, dev->buf, 6);
	*(dev->buf + 6) = crc;
        *(dev->buf + 7) = crc >> 8;
        dev->r_cnt = dev->r_sz = 8;
	int ret;
	if (0 != (ret = wait_trans(dev, adr, req->tmo_ms))) {
		return (ret);
	}
	if (*(dev->buf + 1) == WR_SINGLE_REGISTER) {
		req->except = 0;
	} else if (*(dev->buf + 1) == (WR_SINGLE_REGISTER | 0x80)) {
		req->except = *(dev->buf + 2);
		return (-ENACK);
	} else {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	int tmp = *(dev->buf + 2) << 8;
	tmp |= *(dev->buf + 3);
	if (tmp != radr) {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	tmp = *(dev->buf + 4) << 8;
	tmp |= *(dev->buf + 5);
	if (tmp != req->regs[0]) {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	return (0);
}

/**
 * modbm_read_bits
 */
int modbm_read_bits(modbm dev, int adr, enum modbm_bit_table table, struct modbm_bit_io_req *req)
{
	*dev->buf = adr;
	if (table == MODBM_DISC_INPUT_TABLE) {
		*(dev->buf + 1) = RD_DISCRETE_INPUTS;
	} else if (table == MODBM_COILS_TABLE) {
		*(dev->buf + 1) = RD_COILS;
	} else {
		crit_err_exit(BAD_PARAMETER);
	}
	if (req->bit_cnt < 1 || req->bit_cnt > 2000) {
		crit_err_exit(BAD_PARAMETER);
	}
	int badr = req->bit_start - 1;
	if (badr < 0 || badr + req->bit_cnt - 1 > 0xFFFF) {
		crit_err_exit(BAD_PARAMETER);
	}
	*(dev->buf + 2) = badr >> 8;
        *(dev->buf + 3) = badr;
	*(dev->buf + 4) = req->bit_cnt >> 8;
        *(dev->buf + 5) = req->bit_cnt;
        uint16_t crc = crc_16(INIT_CRC_16_MODB, dev->buf, 6);
	*(dev->buf + 6) = crc;
        *(dev->buf + 7) = crc >> 8;
	dev->s_sz = 8;
	dev->r_sz = req->bit_cnt / 8 + 5;
	if (req->bit_cnt % 8) {
		dev->r_sz++;
	}
	dev->r_cnt = dev->r_sz;
	if (dev->r_sz > dev->buf_len) {
		crit_err_exit(BAD_PARAMETER);
	}
	int ret;
	if (0 != (ret = wait_trans(dev, adr, req->tmo_ms))) {
		return (ret);
	}
	int fn = (table == MODBM_DISC_INPUT_TABLE) ? RD_DISCRETE_INPUTS : RD_COILS;
	if (*(dev->buf + 1) == fn) {
		req->except = 0;
	} else if (*(dev->buf + 1) == (fn | 0x80)) {
		req->except = *(dev->buf + 2);
		return (-ENACK);
	} else {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	int bn = dev->r_sz - 5;
	if (*(dev->buf + 2) != bn) {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	for (int i = 0; i < bn; i++) {
		req->bits[i] = *(dev->buf + 3 + i);
	}
	return (0);
}

/**
 * modbm_write_bits
 */
int modbm_write_bits(modbm dev, int adr, struct modbm_bit_io_req *req)
{
	*dev->buf = adr;
	*(dev->buf + 1) = WR_MULTIPLE_COILS;
	if (req->bit_cnt < 1 || req->bit_cnt > 1968) {
		crit_err_exit(BAD_PARAMETER);
	}
	int badr = req->bit_start - 1;
	if (badr < 0 || badr + req->bit_cnt - 1 > 0xFFFF) {
		crit_err_exit(BAD_PARAMETER);
	}
	*(dev->buf + 2) = badr >> 8;
        *(dev->buf + 3) = badr;
	*(dev->buf + 4) = req->bit_cnt >> 8;
        *(dev->buf + 5) = req->bit_cnt;
	int bn = req->bit_cnt / 8;
	if (req->bit_cnt % 8) {
		bn++;
	}
	dev->s_sz = bn + 9;
	if (dev->s_sz > dev->buf_len) {
		crit_err_exit(BAD_PARAMETER);
	}
	*(dev->buf + 6) = bn;
	for (int i = 0; i < bn; i++) {
		*(dev->buf + 7 + i) = req->bits[i];
	}
	uint16_t crc = crc_16(INIT_CRC_16_MODB, dev->buf, dev->s_sz - 2);
	*(dev->buf + 7 + bn) = crc;
        *(dev->buf + 7 + bn + 1) = crc >> 8;
	dev->r_cnt = dev->r_sz = 8;
	int ret;
	if (0 != (ret = wait_trans(dev, adr, req->tmo_ms))) {
		return (ret);
	}
	if (*(dev->buf + 1) == WR_MULTIPLE_COILS) {
		req->except = 0;
	} else if (*(dev->buf + 1) == (WR_MULTIPLE_COILS | 0x80)) {
		req->except = *(dev->buf + 2);
		return (-ENACK);
	} else {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	int tmp = *(dev->buf + 2) << 8;
	tmp |= *(dev->buf + 3);
	if (tmp != badr) {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	tmp = *(dev->buf + 4) << 8;
	tmp |= *(dev->buf + 5);
	if (tmp != req->bit_cnt) {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	return (0);
}

/**
 * modbm_write_bit
 */
int modbm_write_bit(modbm dev, int adr, struct modbm_bit_io_req *req)
{
	*dev->buf = adr;
	*(dev->buf + 1) = WR_SINGLE_COIL;
	int badr = req->bit_start - 1;
	if (badr < 0 || badr > 0xFFFF) {
		crit_err_exit(BAD_PARAMETER);
	}
	*(dev->buf + 2) = badr >> 8;
        *(dev->buf + 3) = badr;
	*(dev->buf + 4) = (req->bits[0] & 1) ? 0xFF : 0x00;
        *(dev->buf + 5) = 0x00;
	uint16_t crc = crc_16(INIT_CRC_16_MODB, dev->buf, 6);
	*(dev->buf + 6) = crc;
        *(dev->buf + 7) = crc >> 8;
	dev->s_sz = 8;
	dev->r_cnt = dev->r_sz = 8;
	int ret;
	if (0 != (ret = wait_trans(dev, adr, req->tmo_ms))) {
		return (ret);
	}
	if (*(dev->buf + 1) == WR_SINGLE_COIL) {
		req->except = 0;
	} else if (*(dev->buf + 1) == (WR_SINGLE_COIL | 0x80)) {
		req->except = *(dev->buf + 2);
		return (-ENACK);
	} else {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
	int tmp = *(dev->buf + 2) << 8;
	tmp |= *(dev->buf + 3);
	if (tmp != badr) {
		dev->stats.fmt_err++;
		return (-EGEN);
	}
        return (0);
}

/**
 * wait_trans
 */
static int wait_trans(modbm dev, int adr, int tmo)
{
        reg_sercom_isr_clbk(dev->irqn, tx_hndlr, dev);
	dev->s_sz--;
	dev->p = dev->buf + 1;
	dev->de_pin_ctl(HIGH);
        dev->mmio->DR = *dev->buf;
        barrier();
        dev->mmio->CR1 |= USART_CR1_TXEIE;
        uint8_t q;
        if (pdFALSE == xQueueReceive(dev->sig_que, &q, tmo / portTICK_PERIOD_MS)) {
		dis_rx(dev);
                if (dev->wait_idle) {
			dev->wait_idle();
		}
		xQueueReceive(dev->sig_que, &q, 0);
		if (dev->r_cnt == dev->r_sz) {
                        return (-ETMO);
		}
		if (dev->r_cnt == 0) {
			dev->stats.idle_err++;
                        return (-EGEN);
		}
		dev->stats.short_msg++;
		return (-EGEN);
	}
	if (dev->wait_idle) {
		dev->wait_idle();
	}
	if (q == ERCV) {
		return (-EGEN);
	}
	uint16_t crc = *(dev->buf + dev->r_sz - 1) << 8;
        crc |= *(dev->buf + dev->r_sz - 2);
	if (crc != crc_16(INIT_CRC_16_MODB, dev->buf, dev->r_sz - 2)) {
		dev->stats.crc_err++;
		return (-EGEN);
	}
	if (*dev->buf != adr) {
		dev->stats.bad_adr++;
		return (-EGEN);
	}
	return (0);
}

/**
 * dis_rx
 */
static void dis_rx(modbm dev)
{
	unsigned int tmp;

	dev->mmio->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_IDLEIE);
	tmp = dev->mmio->SR;
	tmp = dev->mmio->DR;
        dev->mmio->SR = 0;
	dev->mmio->CR1 &= ~USART_CR1_RE;
	while (dev->mmio->CR1 & USART_CR1_RE);
}

/**
 * tx_hndlr
 */
static BaseType_t tx_hndlr(void *dev)
{
	unsigned int sr = ((modbm) dev)->mmio->SR;

	if (sr & USART_SR_TXE && ((modbm) dev)->mmio->CR1 & USART_CR1_TXEIE) {
		((modbm) dev)->mmio->DR = *((modbm) dev)->p++;
		if (--((modbm) dev)->s_sz == 0) {
			unsigned int r = ((modbm) dev)->mmio->CR1;
			((modbm) dev)->mmio->CR1 = (r & ~USART_CR1_TXEIE) | USART_CR1_TCIE;
		}
		return (pdFALSE);
	} else if (sr & USART_SR_TC && ((modbm) dev)->mmio->CR1 & USART_CR1_TCIE) {
		((modbm) dev)->mmio->CR1 &= ~USART_CR1_TCIE;
                ((modbm) dev)->mmio->CR1 |= USART_CR1_RE;
                ((modbm) dev)->mmio->CR1 |= USART_CR1_RXNEIE;
		((modbm) dev)->p = ((modbm) dev)->buf;
		reg_sercom_isr_clbk(((modbm) dev)->irqn, rx_hndlr, dev);
                ((modbm) dev)->de_pin_ctl(LOW);
		return (pdFALSE);
	}
        ((modbm) dev)->stats.unexp_intr++;
        return (pdFALSE);
}

/**
 * rx_hndlr
 */
static BaseType_t rx_hndlr(void *dev)
{
	unsigned int sr = ((modbm) dev)->mmio->SR;

        if (sr & USART_SR_RXNE && ((modbm) dev)->mmio->CR1 & USART_CR1_RXNEIE) {
		*((modbm) dev)->p++ = ((modbm) dev)->mmio->DR;
		if (sr & 0xF) {
			((modbm) dev)->mmio->CR1 &= ~USART_CR1_RXNEIE;
                        ((modbm) dev)->mmio->CR1 &= ~USART_CR1_RE;
			if (sr & USART_SR_PE) {
				((modbm) dev)->stats.par_lerr++;
			} else if (sr & USART_SR_FE) {
				((modbm) dev)->stats.fra_lerr++;
                        } else if (sr & USART_SR_NE) {
				((modbm) dev)->stats.noi_lerr++;
                        } else if (sr & USART_SR_ORE) {
				((modbm) dev)->stats.ovr_lerr++;
			}
			uint8_t q = ERCV;
                        BaseType_t wkn = pdFALSE;
			xQueueSendFromISR(((modbm) dev)->sig_que, &q, &wkn);
			return (wkn);
		}
		if (--((modbm) dev)->r_cnt == 0) {
			unsigned int r = ((modbm) dev)->mmio->CR1;
			((modbm) dev)->mmio->CR1 = (r & ~USART_CR1_RXNEIE) | USART_CR1_IDLEIE;
			reg_sercom_isr_clbk(((modbm) dev)->irqn, idle_hndlr, dev);
                        return (pdFALSE);
		}
		if (((modbm) dev)->r_sz - ((modbm) dev)->r_cnt == 2) {
			if (*(((modbm) dev)->buf + 1) & 0x80) {
				((modbm) dev)->r_sz = 5;
                                ((modbm) dev)->r_cnt = 3;
			}
		}
		return (pdFALSE);
	}
        ((modbm) dev)->stats.unexp_intr++;
        return (pdFALSE);
}

/**
 * idle_hndlr
 */
static BaseType_t idle_hndlr(void *dev)
{
	unsigned int sr = ((modbm) dev)->mmio->SR;

	if (sr & USART_SR_IDLE && ((modbm) dev)->mmio->CR1 & USART_CR1_IDLEIE) {
		unsigned int tmp;
		tmp = ((modbm) dev)->mmio->DR;
                ((modbm) dev)->mmio->CR1 &= ~USART_CR1_IDLEIE;
                ((modbm) dev)->mmio->CR1 &= ~USART_CR1_RE;
		uint8_t q = 0;
                BaseType_t wkn = pdFALSE;
		xQueueSendFromISR(((modbm) dev)->sig_que, &q, &wkn);
		return (wkn);
	}
        ((modbm) dev)->stats.unexp_intr++;
        return (pdFALSE);
}

/**
 * reg_brr_val
 */
static unsigned int reg_brr_val(modbm dev)
{
	int clk;
	unsigned int ret;

	if (dev->apb_inst == CLK_LOW_SPEED_APB1) {
		clk = clk_freq.pclk1;
	} else {
		clk = clk_freq.pclk2;
	}
	ret = clk / dev->baudrate;
	if (!(ret & 0xFFF0)) {
		crit_err_exit(BAD_PARAMETER);
	}
	return (ret);
}

#if TERMOUT == 1
/**
 * log_modbm_errors
 */
void log_modbm_errors(modbm dev)
{
	msg(INF, "modbm.c: unexp_intr=%d par_lerr=%d fra_lerr=%d\n",
	    dev->stats.unexp_intr, dev->stats.par_lerr, dev->stats.fra_lerr);
	msg(INF, "modbm.c: noi_lerr=%d ovr_lerr=%d short_msg=%d\n",
	    dev->stats.noi_lerr, dev->stats.ovr_lerr, dev->stats.short_msg);
	msg(INF, "modbm.c: crc_err=%d bad_adr=%d fmt_err=%d\n",
	    dev->stats.crc_err, dev->stats.bad_adr, dev->stats.fmt_err);
	msg(INF, "modbm.c: idle_err=%d\n", dev->stats.idle_err);
}
#endif

#endif
