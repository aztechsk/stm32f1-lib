/*
 * sercom.c
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
#include "tools.h"
#include "hwerr.h"
#include "clk.h"
#include "sercom_isr.h"
#include "sercom.h"

#if SERCOM_HDLC == 1
enum {
	HDLC_RCV_WAIT_ADDR,
        HDLC_RCV_FLAG_1,
        HDLC_RCV_DATA,
        HDLC_RCV_ESC
};
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
static unsigned int reg_brr_val(sercom dev);
#endif
#if SERCOM_RX_CHAR == 1
static BaseType_t rx_char_hndlr(void *dev);
#endif
#if SERCOM_HDLC == 1
static BaseType_t hdlc_hndlr(void *dev);
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
/**
 * init_sercom
 */
void init_sercom(sercom dev, enum sercom_mode m)
{
#ifdef USART1
	if (dev->instance == SERCOM_USART1) {
		dev->irqn = USART1_IRQn;
                dev->apb_inst = CLK_HIGH_SPEED_APB2;
                dev->apb_bmp =  RCC_APB2ENR_USART1EN;
                dev->mmio = USART1;
		goto lb1;
	}
#endif
#ifdef USART2
	if (dev->instance == SERCOM_USART2) {
		dev->irqn = USART2_IRQn;
                dev->apb_inst = CLK_LOW_SPEED_APB1;
                dev->apb_bmp = RCC_APB1ENR_USART2EN;
                dev->mmio = USART2;
		goto lb1;
	}
#endif
#ifdef USART3
	if (dev->instance == SERCOM_USART3) {
		dev->irqn = USART3_IRQn;
                dev->apb_inst = CLK_LOW_SPEED_APB1;
                dev->apb_bmp = RCC_APB1ENR_USART3EN;
                dev->mmio = USART3;
		goto lb1;
	}
#endif
	crit_err_exit(BAD_PARAMETER);
lb1:
	dev->mode = m;
	switch (m) {
#if SERCOM_RX_CHAR == 1
	case SERCOM_RX_CHAR_MODE :
		if (dev->rx_que == NULL) {
			if (NULL == (dev->rx_que = xQueueCreate(dev->rx_que_size, sizeof(uint16_t)))) {
				crit_err_exit(MALLOC_ERROR);
			}
		} else {
			crit_err_exit(UNEXP_PROG_STATE);
		}
		reg_sercom_isr_clbk(dev->irqn, rx_char_hndlr, dev);
		break;
#endif
#if SERCOM_HDLC == 1
	case SERCOM_HDLC_MODE :
		if (NULL == (dev->hdlc_mesg.pld = pvPortMalloc(dev->hdlc_bf_sz))) {
			crit_err_exit(MALLOC_ERROR);
		}
		if (dev->hdlc_rx_sig == NULL) {
			if (NULL == (dev->hdlc_rx_sig = xQueueCreate(1, sizeof(uint8_t)))) {
				crit_err_exit(MALLOC_ERROR);
			}
		} else {
			crit_err_exit(UNEXP_PROG_STATE);
		}
		reg_sercom_isr_clbk(dev->irqn, hdlc_hndlr, dev);
		break;
#endif
	default :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
	dev->reg_brr = reg_brr_val(dev);
	dev->reg_cr1 = 0;
#if SERCOM_RX_CHAR == 1
	if (dev->char_size == SERCOM_CHAR_9_BITS) {
		dev->reg_cr1 |= USART_CR1_M;
	}
#endif
	if (dev->parity != SERCOM_PARITY_NO) {
		dev->reg_cr1 |= USART_CR1_PCE;
	}
	if (dev->parity == SERCOM_PARITY_ODD) {
		dev->reg_cr1 |= USART_CR1_PS;
	}
	dev->reg_cr2 = 0;
	if (dev->stop_bit == SERCOM_2_STOP_BITS) {
		dev->reg_cr2 |= 2 << USART_CR2_STOP_Pos;
	}
        dev->reg_cr3 = 0;
#if SERCOM_RX_CHAR == 1
	if (dev->char_size == SERCOM_CHAR_9_BITS && dev->parity == SERCOM_PARITY_NO) {
		dev->b16 = TRUE;
	} else {
		dev->b16 = FALSE;
	}
#endif
        NVIC_DisableIRQ(dev->irqn);
        enable_per_apb_clk(dev->apb_inst, dev->apb_bmp);
	dev->mmio->CR1 = dev->reg_cr1 | USART_CR1_UE;
	while (!(dev->mmio->CR1 & USART_CR1_UE));
	dev->mmio->CR2 = dev->reg_cr2;
	dev->mmio->CR3 = dev->reg_cr3;
	dev->mmio->BRR = dev->reg_brr;
	dev->mmio->SR = 0;
        dev->conf_pins(SERCOM_PINS_PERIPHERAL);
        NVIC_SetPriority(dev->irqn, configLIBRARY_MAX_API_CALL_INTERRUPT_PRIORITY);
        NVIC_ClearPendingIRQ(dev->irqn);
	NVIC_EnableIRQ(dev->irqn);
}
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
/**
 * enable_sercom
 */
void enable_sercom(void *dev)
{
	enable_per_apb_clk(((sercom) dev)->apb_inst, ((sercom) dev)->apb_bmp);
	((sercom) dev)->mmio->CR1 = ((sercom) dev)->reg_cr1 | USART_CR1_UE;
	while (!(((sercom) dev)->mmio->CR1 & USART_CR1_UE));
	((sercom) dev)->mmio->CR2 = ((sercom) dev)->reg_cr2;
	((sercom) dev)->mmio->CR3 = ((sercom) dev)->reg_cr3;
	((sercom) dev)->mmio->BRR = ((sercom) dev)->reg_brr;
	((sercom) dev)->mmio->SR = 0;
        ((sercom) dev)->conf_pins(SERCOM_PINS_PERIPHERAL);
	switch (((sercom) dev)->mode) {
#if SERCOM_RX_CHAR == 1
	case SERCOM_RX_CHAR_MODE :
		{
			uint16_t u16;
			while (pdTRUE == xQueueReceive(((sercom) dev)->rx_que, &u16, 0));
		}
		break;
#endif
#if SERCOM_HDLC == 1
	case SERCOM_HDLC_MODE :
		{
			uint8_t u8;
			while (pdTRUE == xQueueReceive(((sercom) dev)->hdlc_rx_sig, &u8, 0));
		}
		break;
#endif
	default :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
        NVIC_ClearPendingIRQ(((sercom) dev)->irqn);
	NVIC_EnableIRQ(((sercom) dev)->irqn);
}
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
/**
 * disable_sercom
 */
void disable_sercom(void *dev)
{
	NVIC_DisableIRQ(((sercom) dev)->irqn);
	((sercom) dev)->conf_pins(SERCOM_PINS_GPIO);
	((sercom) dev)->mmio->CR1 &= ~USART_CR1_TE;
	while (((sercom) dev)->mmio->CR1 & USART_CR1_TE);
	((sercom) dev)->mmio->CR1 &= ~USART_CR1_RE;
	while (((sercom) dev)->mmio->CR1 & USART_CR1_RE);
	((sercom) dev)->mmio->CR1 &= ~USART_CR1_UE;
	while (((sercom) dev)->mmio->CR1 & USART_CR1_UE);
	disable_per_apb_clk(((sercom) dev)->apb_inst, ((sercom) dev)->apb_bmp);
}
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
/**
 * sercom_tx_buff
 */
int sercom_tx_buff(void *dev, void *p_buf, int size)
{
	int ret = 0;

	if (size < 1) {
                crit_err_exit(BAD_PARAMETER);
        }
	if (!(((sercom) dev)->mmio->CR1 & USART_CR1_TE)) {
		taskENTER_CRITICAL();
		((sercom) dev)->mmio->CR1 |= USART_CR1_TE;
                taskEXIT_CRITICAL();
	}
	while (!(((sercom) dev)->mmio->SR & USART_SR_TXE));
        ((sercom) dev)->size = --size;
#if SERCOM_RX_CHAR == 1
	if (((sercom) dev)->b16) {
		((sercom) dev)->mmio->DR = *((uint16_t *) p_buf);
                ((sercom) dev)->p_buf = (uint16_t *) p_buf + 1;
	} else {
#endif
		((sercom) dev)->mmio->DR = *((uint8_t *) p_buf);
                ((sercom) dev)->p_buf = (uint8_t *) p_buf + 1;
#if SERCOM_RX_CHAR == 1
	}
#endif
	((sercom) dev)->c_tsk = xTaskGetCurrentTaskHandle();
	taskENTER_CRITICAL();
        if (size) {
		((sercom) dev)->mmio->CR1 |= USART_CR1_TXEIE;
	} else {
		((sercom) dev)->mmio->CR1 |= USART_CR1_TCIE;
	}
        taskEXIT_CRITICAL();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#if SERCOM_TRANS_IDLE_OFF == 1
	taskENTER_CRITICAL();
	((sercom) dev)->mmio->CR1 &= ~USART_CR1_TE;
	taskEXIT_CRITICAL();
        while (((sercom) dev)->mmio->CR1 & USART_CR1_TE);
#endif
	return (ret);
}
#endif

#if SERCOM_RX_CHAR == 1
/**
 * sercom_rx_char
 */
int sercom_rx_char(void *dev, void *p_char, TickType_t tmo)
{
	uint16_t d;

	if (!(((sercom) dev)->mmio->CR1 & USART_CR1_RE)) {
		((sercom) dev)->sr_pe = FALSE;
		taskENTER_CRITICAL();
		((sercom) dev)->mmio->CR1 |= USART_CR1_RE;
		while (!(((sercom) dev)->mmio->CR1 & USART_CR1_RE));
		((sercom) dev)->mmio->CR1 |= USART_CR1_RXNEIE;
                taskEXIT_CRITICAL();
	}
	if (pdFALSE == xQueueReceive(((sercom) dev)->rx_que, &d, tmo)) {
		if (((sercom) dev)->b16) {
			*((uint16_t *) p_char) = '\0';
		} else {
			*((uint8_t *) p_char) = '\0';
		}
		return (-ETMO);
	}
        if (d & 0xF800) {
		if (d & 0x0800) {
			if (((sercom) dev)->b16) {
				*((uint16_t *) p_char) = '\0';
			} else {
				*((uint8_t *) p_char) = '\0';
			}
			return (-EINTR);
		} else {
			d >>= 12;
			if (((sercom) dev)->b16) {
				*((uint16_t *) p_char) = d;
			} else {
				*((uint8_t *) p_char) = d;
			}
			return (-ERCV);
		}
	} else {
		if (((sercom) dev)->b16) {
			*((uint16_t *) p_char) = d;
		} else {
			*((uint8_t *) p_char) = d;
		}
                return (0);
	}
}
#endif

#if SERCOM_HDLC == 1
/**
 * sercom_tx_hdlc_mesg
 */
int sercom_tx_hdlc_mesg(sercom dev, uint8_t *p_pld, int size)
{
        int sz = 1;

	if (size < 1) {
                crit_err_exit(BAD_PARAMETER);
        }
	*dev->hdlc_mesg.pld = dev->hdlc_flag;
        for (int i = 0; i < size; i++) {
		if (*(p_pld + i) == dev->hdlc_flag || *(p_pld + i) == dev->hdlc_esc) {
			if (sz + 2 < dev->hdlc_bf_sz) {
				*(dev->hdlc_mesg.pld + sz++) = dev->hdlc_esc;
				*(dev->hdlc_mesg.pld + sz++) = *(p_pld + i) ^ dev->hdlc_mod;
			} else {
				return (-EBFOV);
			}
		} else {
			if (sz + 1 < dev->hdlc_bf_sz) {
				*(dev->hdlc_mesg.pld + sz++) = *(p_pld + i);
			} else {
				return (-EBFOV);
			}
		}
	}
	*(dev->hdlc_mesg.pld + sz++) = dev->hdlc_flag;
	return (sercom_tx_buff(dev, dev->hdlc_mesg.pld, sz));
}

/**
 * sercom_rx_hdlc_mesg
 */
struct hdlc_mesg *sercom_rx_hdlc_mesg(sercom dev, TickType_t tmo)
{
	unsigned int tmp;
        uint8_t u8;

	taskENTER_CRITICAL();
	dev->mmio->CR1 |= USART_CR1_RE;
	taskEXIT_CRITICAL();
	while (!(dev->mmio->CR1 & USART_CR1_RE));
        dev->rcv_st = HDLC_RCV_FLAG_1;
	dev->sr_pe = FALSE;
	taskENTER_CRITICAL();
	((sercom) dev)->mmio->CR1 |= USART_CR1_RXNEIE;
	taskEXIT_CRITICAL();
	if (pdFALSE == xQueueReceive(((sercom) dev)->hdlc_rx_sig, &u8, tmo) || u8 == EINTR) {
		taskENTER_CRITICAL();
		dev->mmio->CR1 &= ~USART_CR1_RXNEIE;
		tmp = dev->mmio->SR;
                tmp = dev->mmio->DR;
		dev->mmio->SR = 0;
                dev->mmio->CR1 &= ~USART_CR1_RE;
                taskEXIT_CRITICAL();
                while (dev->mmio->CR1 & USART_CR1_RE);
		while (pdTRUE == xQueueReceive(((sercom) dev)->hdlc_rx_sig, &u8, 0));
                return (NULL);
	} else {
		while (dev->mmio->CR1 & USART_CR1_RE);
		return (&dev->hdlc_mesg);
	}
}
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
/**
 * sercom_intr_rx
 */
boolean_t sercom_intr_rx(void *dev)
{
	switch (((sercom) dev)->mode) {
#if SERCOM_RX_CHAR == 1
	case SERCOM_RX_CHAR_MODE :
		{
			uint16_t u16 = 0x0800;
			if (pdTRUE == xQueueSend(((sercom) dev)->rx_que, &u16, 0)) {
				return (TRUE);
			} else {
				return (FALSE);
			}
		}
		break;
#endif
#if SERCOM_HDLC == 1
	case SERCOM_HDLC_MODE :
		{
			uint8_t u8 = EINTR;
			if (pdTRUE == xQueueSend(((sercom) dev)->hdlc_rx_sig, &u8, 0)) {
				return (TRUE);
			} else {
				return (FALSE);
			}
		}
		break;
#endif
	default :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
        return (FALSE);
}
#endif

#if SERCOM_RX_CHAR == 1
/**
 * sercom_flush_rx
 */
void sercom_flush_rx(void *dev)
{
	uint16_t d;
	unsigned int tmp;

        taskENTER_CRITICAL();
	((sercom) dev)->mmio->CR1 &= ~USART_CR1_RXNEIE;
	tmp = ((sercom) dev)->mmio->SR;
        tmp = ((sercom) dev)->mmio->DR;
	((sercom) dev)->mmio->SR = 0;
	((sercom) dev)->mmio->CR1 &= ~USART_CR1_RE;
	taskEXIT_CRITICAL();
        while (((sercom) dev)->mmio->CR1 & USART_CR1_RE);
	while (pdTRUE == xQueueReceive(((sercom) dev)->rx_que, &d, 0));
}
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
/**
 * reg_brr_val
 */
static unsigned int reg_brr_val(sercom dev)
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
#endif

#if SERCOM_RX_CHAR == 1
/**
 * rx_char_hndlr
 */
static BaseType_t rx_char_hndlr(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	unsigned int sr = ((sercom) dev)->mmio->SR;

	if (sr & USART_SR_TXE && ((sercom) dev)->mmio->CR1 & USART_CR1_TXEIE) {
		if (((sercom) dev)->b16) {
			((sercom) dev)->mmio->DR = *((uint16_t *) ((sercom) dev)->p_buf);
			((sercom) dev)->p_buf = (uint16_t *) ((sercom) dev)->p_buf + 1;
		} else {
			((sercom) dev)->mmio->DR = *((uint8_t *) ((sercom) dev)->p_buf);
			((sercom) dev)->p_buf = (uint8_t *) ((sercom) dev)->p_buf + 1;
		}
		if (--((sercom) dev)->size == 0) {
			unsigned int r = ((sercom) dev)->mmio->CR1;
			((sercom) dev)->mmio->CR1 = (r & ~USART_CR1_TXEIE) | USART_CR1_TCIE;
		}
	} else if (sr & USART_SR_TC && ((sercom) dev)->mmio->CR1 & USART_CR1_TCIE) {
		((sercom) dev)->mmio->CR1 &= ~USART_CR1_TCIE;
		BaseType_t wkn = pdFALSE;
		vTaskNotifyGiveFromISR(((sercom) dev)->c_tsk, &wkn);
		if (wkn) {
			tsk_wkn = pdTRUE;
		}
	}
	if (sr & USART_SR_PE) { // Errata 2.16.3.
		((sercom) dev)->sr_pe = TRUE;
	}
	if (sr & USART_SR_RXNE && ((sercom) dev)->mmio->CR1 & USART_CR1_RXNEIE) {
		uint16_t d = ((sercom) dev)->mmio->DR;
                d &= 0x01FF;
		if (((sercom) dev)->sr_pe) {
			((sercom) dev)->sr_pe = FALSE;
			sr |= USART_SR_PE;
		}
		if (sr & 0xF) {
			d = (sr & 0xF) << 12;
		}
                BaseType_t wkn = pdFALSE;
		xQueueSendFromISR(((sercom) dev)->rx_que, &d, &wkn);
		if (wkn) {
			tsk_wkn = pdTRUE;
		}
	}
        return (tsk_wkn);
}
#endif

#if SERCOM_HDLC == 1
/**
 * hdlc_hndlr
 */
static BaseType_t hdlc_hndlr(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	unsigned int sr = ((sercom) dev)->mmio->SR;

	if (sr & USART_SR_PE) { // Errata 2.16.3.
		((sercom) dev)->sr_pe = TRUE;
	}
	if (sr & USART_SR_RXNE && ((sercom) dev)->mmio->CR1 & USART_CR1_RXNEIE) {
		uint8_t d = ((sercom) dev)->mmio->DR;
		if (((sercom) dev)->sr_pe) {
			((sercom) dev)->sr_pe = FALSE;
			sr |= USART_SR_PE;
		}
		if (sr & 0xF) {
			if (sr & USART_SR_PE) {
				((sercom) dev)->hdlc_stats.par_lerr++;
			} else if (sr & USART_SR_FE) {
				((sercom) dev)->hdlc_stats.fra_lerr++;
                        } else if (sr & USART_SR_NE) {
				((sercom) dev)->hdlc_stats.noi_lerr++;
                        } else if (sr & USART_SR_ORE) {
				((sercom) dev)->hdlc_stats.ovr_lerr++;
			}
			((sercom) dev)->rcv_st = HDLC_RCV_FLAG_1;
                        return (pdFALSE);
		}
                switch (((sercom) dev)->rcv_st) {
		case HDLC_RCV_FLAG_1 :
			if (d == ((sercom) dev)->hdlc_flag) {
				((sercom) dev)->rcv_st = HDLC_RCV_DATA;
                                ((sercom) dev)->hdlc_mesg.sz = 0;
			} else {
				((sercom) dev)->hdlc_stats.no_f1_perr++;
			}
			break;
		case HDLC_RCV_DATA :
			if (d == ((sercom) dev)->hdlc_flag) {
				if (((sercom) dev)->hdlc_mesg.sz != 0) {
					((sercom) dev)->mmio->CR1 &= ~USART_CR1_RXNEIE;
					((sercom) dev)->mmio->CR1 &= ~USART_CR1_RE;
                                        BaseType_t wkn = pdFALSE;
					uint8_t u8 = 0;
                                        xQueueSendFromISR(((sercom) dev)->hdlc_rx_sig, &u8, &wkn);
					if (wkn) {
						tsk_wkn = pdTRUE;
					}
				} else {
					((sercom) dev)->hdlc_stats.syn_f1_perr++;
				}
			} else if (d == ((sercom) dev)->hdlc_esc) {
				((sercom) dev)->rcv_st = HDLC_RCV_ESC;
			} else {
				if (((sercom) dev)->hdlc_mesg.sz < ((sercom) dev)->hdlc_bf_sz) {
					*(((sercom) dev)->hdlc_mesg.pld + ((sercom) dev)->hdlc_mesg.sz++) = d;
				} else {
					((sercom) dev)->hdlc_stats.bf_ov_perr++;
                                        ((sercom) dev)->rcv_st = HDLC_RCV_FLAG_1;
				}
			}
			break;
		case HDLC_RCV_ESC :
			if (((sercom) dev)->hdlc_mesg.sz < ((sercom) dev)->hdlc_bf_sz) {
				uint8_t n = d ^ ((sercom) dev)->hdlc_mod;
				if (n == ((sercom) dev)->hdlc_flag || n == ((sercom) dev)->hdlc_esc) {
					*(((sercom) dev)->hdlc_mesg.pld + ((sercom) dev)->hdlc_mesg.sz++) = n;
					((sercom) dev)->rcv_st = HDLC_RCV_DATA;
				} else {
					((sercom) dev)->hdlc_stats.es_sq_perr++;
					((sercom) dev)->rcv_st = HDLC_RCV_FLAG_1;
				}
			} else {
				((sercom) dev)->hdlc_stats.bf_ov_perr++;
				((sercom) dev)->rcv_st = HDLC_RCV_FLAG_1;
			}
			break;
		}
	}
	if (sr & USART_SR_TXE && ((sercom) dev)->mmio->CR1 & USART_CR1_TXEIE) {
		((sercom) dev)->mmio->DR = *((uint8_t *) ((sercom) dev)->p_buf);
		((sercom) dev)->p_buf = (uint8_t *) ((sercom) dev)->p_buf + 1;
		if (--((sercom) dev)->size == 0) {
			unsigned int r = ((sercom) dev)->mmio->CR1;
			((sercom) dev)->mmio->CR1 = (r & ~USART_CR1_TXEIE) | USART_CR1_TCIE;
		}
	} else if (sr & USART_SR_TC && ((sercom) dev)->mmio->CR1 & USART_CR1_TCIE) {
		((sercom) dev)->mmio->CR1 &= ~USART_CR1_TCIE;
		BaseType_t wkn = pdFALSE;
                vTaskNotifyGiveFromISR(((sercom) dev)->c_tsk, &wkn);
		if (wkn) {
			tsk_wkn = pdTRUE;
		}
	}
	return (tsk_wkn);
}
#endif
