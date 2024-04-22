/*
 * i2cm.c
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
#include "clk.h"
#include "i2c_isr.h"
#include "i2cm.h"
#include <stdarg.h>

#if I2CM == 1

#define WAIT_INTR (200 / portTICK_PERIOD_MS)

enum evn_wrst {
	EVN_WRST_START,
        EVN_WRST_ADR,
        EVN_WRST_ADR10,
        EVN_WRST_NEXTB,
        EVN_WRST_STOP
};

enum evn_iadrst {
	EVN_IADRST_START,
        EVN_IADRST_ADR,
        EVN_IADRST_ADR10,
        EVN_IADRST_NEXTB
};

enum evn_rdst_a7 {
	EVN_RDST_A7_START,
        EVN_RDST_A7_ADR,
};

enum evn_rdst_a10 {
	EVN_RDST_A10_START,
        EVN_RDST_A10_ADR10,
        EVN_RDST_A10_ADR,
        EVN_RDST_A10_N_START,
        EVN_RDST_A10_N_HEAD
};

enum evn_rdst_bn {
	EVN_RDST_BN_3B,
	EVN_RDST_BN_NB,
        EVN_RDST_BN_BTF,
	EVN_RDST_BN_LAST
};

static BaseType_t evn_wr_stm(void *dev);
static BaseType_t evn_iadr_stm(void *dev);
static BaseType_t evn_rd_stm_a7(void *dev);
static BaseType_t evn_rd_stm_a10(void *dev);
static boolean_t rd_start(void *dev);
static BaseType_t evn_rd_stm_b1(void *dev);
static BaseType_t evn_rd_stm_b2(void *dev);
static BaseType_t evn_rd_stm_bn(void *dev);
static BaseType_t ehw_sig(void *dev);
static BaseType_t err(void *dev);

/**
 * init_i2cm
 */
void init_i2cm(i2cm dev)
{
	int freq, t_speed_ns, t_pclk_ns;

	if (dev->use_mtx) {
		if (NULL == (dev->mtx = xSemaphoreCreateMutex())) {
			crit_err_exit(MALLOC_ERROR);
		}
	}
#ifdef I2C1
	if (dev->mmio == I2C1) {
		dev->irqn_ev = I2C1_EV_IRQn;
		dev->irqn_er = I2C1_ER_IRQn;
                dev->apb_bmp = RCC_APB1ENR_I2C1EN;
		goto lb1;
	}
#endif
#ifdef I2C2
	if (dev->mmio == I2C2) {
		dev->irqn_ev = I2C2_EV_IRQn;
		dev->irqn_er = I2C2_ER_IRQn;
                dev->apb_bmp = RCC_APB1ENR_I2C2EN;
		goto lb1;
	}
#endif
	crit_err_exit(BAD_PARAMETER);
lb1:
	if (dev->sig_que == NULL) {
		dev->sig_que = xQueueCreate(1, sizeof(uint8_t));
		if (dev->sig_que == NULL) {
			crit_err_exit(MALLOC_ERROR);
		}
	} else {
		crit_err_exit(UNEXP_PROG_STATE);
	}
	freq = clk_freq.pclk1 / 1000000;
	dev->reg_cr2 = freq;
        dev->reg_trise = (dev->speed <= 100000) ? freq + 1 : (freq * 300 / 1000) + 1;
	t_speed_ns = 1000000000 / dev->speed;
	t_pclk_ns = 1000000000 / clk_freq.pclk1;
	if (dev->mode == I2C_MODE_STD) {
		dev->reg_ccr = t_speed_ns / (2 * t_pclk_ns);
	} else {
		if (dev->duty == I2C_FM_DUTY_2) {
			dev->reg_ccr = t_speed_ns / (3 * t_pclk_ns);
		} else {
			dev->reg_ccr = t_speed_ns / (25 * t_pclk_ns);
			dev->reg_ccr |= I2C_CCR_DUTY;
		}
		dev->reg_ccr |= I2C_CCR_FS;
	}
        NVIC_DisableIRQ(dev->irqn_ev);
        NVIC_DisableIRQ(dev->irqn_er);
        enable_per_apb_clk(CLK_LOW_SPEED_APB1, dev->apb_bmp);
	dev->mmio->CR1 |= I2C_CR1_SWRST;
        dev->mmio->CR1 &= ~I2C_CR1_SWRST;
	dev->mmio->CR2 = dev->reg_cr2;
	dev->mmio->TRISE = dev->reg_trise;
	dev->mmio->CCR = dev->reg_ccr;
        dev->conf_pins(I2C_PINS_PERIPHERAL);
        dev->mmio->CR1 = I2C_CR1_PE;
        while (!(dev->mmio->CR1 & I2C_CR1_PE));
        NVIC_SetPriority(dev->irqn_ev, configLIBRARY_MAX_API_CALL_INTERRUPT_PRIORITY);
        NVIC_SetPriority(dev->irqn_er, configLIBRARY_MAX_API_CALL_INTERRUPT_PRIORITY);
        NVIC_ClearPendingIRQ(dev->irqn_ev);
        NVIC_ClearPendingIRQ(dev->irqn_er);
	NVIC_EnableIRQ(dev->irqn_ev);
	NVIC_EnableIRQ(dev->irqn_er);
}

/**
 * lock_i2c_bus
 */
void lock_i2c_bus(i2cm dev)
{
	xSemaphoreTake(dev->mtx, portMAX_DELAY);
}

/**
 * unlock_i2c_bus
 */
void unlock_i2c_bus(i2cm dev)
{
	xSemaphoreGive(dev->mtx);
}

/**
 * reset_i2c_bus
 */
void reset_i2c_bus(i2cm dev)
{
	dev->conf_pins(I2C_PINS_OPEN_DRAIN);
	for (int i = 0; i < 9; i++) {
		dev->conf_pins(I2C_PIN_SCL_LOW);
                vTaskDelay(I2CM_RST_CLK4_MS * 2 / portTICK_PERIOD_MS);
                dev->conf_pins(I2C_PIN_SCL_HIGH);
                vTaskDelay(I2CM_RST_CLK4_MS * 2 / portTICK_PERIOD_MS);
	}
	dev->conf_pins(I2C_PIN_SCL_LOW);
	vTaskDelay(I2CM_RST_CLK4_MS / portTICK_PERIOD_MS);
	dev->conf_pins(I2C_PIN_SDA_LOW);
        vTaskDelay(I2CM_RST_CLK4_MS / portTICK_PERIOD_MS);
        dev->conf_pins(I2C_PIN_SCL_HIGH);
	vTaskDelay(I2CM_RST_CLK4_MS / portTICK_PERIOD_MS);
        dev->conf_pins(I2C_PIN_SDA_HIGH);
	vTaskDelay(I2CM_RST_CLK4_MS / portTICK_PERIOD_MS);
	dev->conf_pins(I2C_PINS_PERIPHERAL);
}

/**
 * enable_i2cm
 */
void enable_i2cm(i2cm dev)
{
	uint8_t u8;

	enable_per_apb_clk(CLK_LOW_SPEED_APB1, dev->apb_bmp);
	dev->mmio->CR1 |= I2C_CR1_SWRST;
        dev->mmio->CR1 &= ~I2C_CR1_SWRST;
	dev->mmio->CR2 = dev->reg_cr2;
	dev->mmio->TRISE = dev->reg_trise;
	dev->mmio->CCR = dev->reg_ccr;
        dev->conf_pins(I2C_PINS_PERIPHERAL);
        dev->mmio->CR1 = I2C_CR1_PE;
        while (!(dev->mmio->CR1 & I2C_CR1_PE));
        xQueueReceive(dev->sig_que, &u8, 0);
        NVIC_ClearPendingIRQ(dev->irqn_ev);
        NVIC_ClearPendingIRQ(dev->irqn_er);
	NVIC_EnableIRQ(dev->irqn_ev);
	NVIC_EnableIRQ(dev->irqn_er);
}

/**
 * disable_i2cm
 */
void disable_i2cm(i2cm dev)
{
        NVIC_DisableIRQ(dev->irqn_ev);
        NVIC_DisableIRQ(dev->irqn_er);
        dev->conf_pins(I2C_PINS_GPIO);
	dev->mmio->CR1 &= ~I2C_CR1_PE;
        while (dev->mmio->CR1 & I2C_CR1_PE);
	dev->mmio->CR1 |= I2C_CR1_SWRST;
        dev->mmio->CR1 &= ~I2C_CR1_SWRST;
	disable_per_apb_clk(CLK_LOW_SPEED_APB1, dev->apb_bmp);
}

/**
 * i2cm_io
 */
int i2cm_io(i2cm dev, enum i2cm_oper op, int adr, uint8_t *p_buf, int size, ...)
{
	uint8_t msg;
        volatile unsigned int tmp;
        va_list ap;

	if (size < 1) {
		crit_err_exit(BAD_PARAMETER);
	}
	if (dev->mmio->SR2 & I2C_SR2_BUSY) {
		msg = EBUSY;
		goto ex;
	}
	if (op > I2CM_WRITE_ADR10) {
		int iadr;
		va_start(ap, size);
		iadr = va_arg(ap, int);
		va_end(ap);
		switch (op) {
		case I2CM_READ_ADR7_IADR1 :
			dev->iadr_sz = 1;
                        op = I2CM_READ_ADR7;
			break;
		case I2CM_READ_ADR7_IADR2 :
                	dev->iadr_sz = 2;
                        op = I2CM_READ_ADR7;
			break;
		case I2CM_READ_ADR10_IADR1 :
			dev->iadr_sz = 1;
			op = I2CM_READ_ADR10;
			break;
		case I2CM_READ_ADR10_IADR2 :
                	dev->iadr_sz = 2;
                        op = I2CM_READ_ADR10;
			break;
		case I2CM_WRITE_ADR7_IADR1 :
			dev->iadr_sz = 1;
                        op = I2CM_WRITE_ADR7;
			break;
		case I2CM_WRITE_ADR7_IADR2 :
                	dev->iadr_sz = 2;
			op = I2CM_WRITE_ADR7;
			break;
		case I2CM_WRITE_ADR10_IADR1 :
			dev->iadr_sz = 1;
                        op = I2CM_WRITE_ADR10;
			break;
		case I2CM_WRITE_ADR10_IADR2 :
                	dev->iadr_sz = 2;
                        op = I2CM_WRITE_ADR10;
			break;
		default :
			crit_err_exit(BAD_PARAMETER);
			break;
		}
                dev->iadr_idx = 0;
		for (int i = dev->iadr_sz - 1; i >= 0; i--) {
			dev->iadr[dev->iadr_sz - 1 - i] = iadr >> i * 8;
		}
	} else {
		dev->iadr_sz = 0;
	}
	if (op == I2CM_READ_ADR7 || op == I2CM_WRITE_ADR7) {
		dev->adb1 = adr << 1;
	} else {
		dev->adb1 = 0xF0 | ((adr & 0x0300) >> 7);
		dev->adb2 = adr;
	}
        dev->sz = size;
	dev->buf = p_buf;
	dev->op = op;
        dev->sr1_err = 0;
	dev->mmio->SR1 = 0;
	dev->mmio->CR1 |= I2C_CR1_ACK;
        dev->mmio->CR1 &= ~I2C_CR1_POS;
        dev->mmio->CR2 &= ~I2C_CR2_ITBUFEN;
	if (op == I2CM_READ_ADR7 || op == I2CM_READ_ADR10) {
		if (dev->iadr_sz) {
			reg_i2c_isr_clbks(dev->mmio, evn_iadr_stm, err, dev);
			dev->st = EVN_IADRST_START;
		} else {
			if (op == I2CM_READ_ADR7) {
				reg_i2c_isr_clbks(dev->mmio, evn_rd_stm_a7, err, dev);
				dev->st = EVN_RDST_A7_START;
			} else {
				reg_i2c_isr_clbks(dev->mmio, evn_rd_stm_a10, err, dev);
				dev->st = EVN_RDST_A10_START;
			}
		}
	} else {
		reg_i2c_isr_clbks(dev->mmio, evn_wr_stm, err, dev);
		dev->st = EVN_WRST_START;
	}
        taskENTER_CRITICAL();
        dev->mmio->CR1 |= I2C_CR1_START;
	dev->mmio->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
        taskEXIT_CRITICAL();
        if (pdFALSE == xQueueReceive(dev->sig_que, &msg, WAIT_INTR)) {
		dev->mmio->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
                tmp = dev->mmio->SR1;
		tmp = dev->mmio->SR2;
		dev->mmio->SR1 = 0;
		dev->stm_tmo_err++;
		msg = EHW;
                goto ex;
	}
	if (msg != 0) {
		if (msg == EGEN) {
			if (dev->sr1_err & I2C_SR1_AF) {
				msg = ENACK;
			} else if (dev->sr1_err & I2C_SR1_ARLO) {
				msg = EACC;
			} else if (dev->sr1_err & I2C_SR1_BERR) {
				dev->bus_err++;
				msg = EHW;
                                goto ex;
			} else if (dev->sr1_err & I2C_SR1_OVR) {
				dev->ovr_err++;
				msg = EHW;
                                goto ex;
			} else {
				crit_err_exit(UNEXP_PROG_STATE);
			}
		} else if (msg == EHW) {
			dev->stm_hw_err++;
                        goto ex;
		} else {
			crit_err_exit(UNEXP_PROG_STATE);
		}
	} else {
		if (dev->sz) {
			crit_err_exit(UNEXP_PROG_STATE);
		}
	}
	tmp = 0;
	while (dev->mmio->SR2 & I2C_SR2_MSL) {
		if (++tmp == 100000) {
			dev->msl_err++;
			break;
		}
	}
ex:
	return (-msg);
}

/**
 * evn_wr_stm
 */
static BaseType_t evn_wr_stm(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	uint8_t msg;
	unsigned int sr1, sr2;

	sr1 = ((i2cm) dev)->mmio->SR1;
	switch (((i2cm) dev)->st) {
	case EVN_WRST_START :
		if (!(sr1 & I2C_SR1_SB)) {
			return (ehw_sig(dev));
		}
		if (((i2cm) dev)->op == I2CM_WRITE_ADR7) {
			((i2cm) dev)->st = EVN_WRST_ADR;
		} else {
			((i2cm) dev)->st = EVN_WRST_ADR10;
		}
		((i2cm) dev)->mmio->DR = ((i2cm) dev)->adb1;
		break;
	case EVN_WRST_ADR :
		sr2 = ((i2cm) dev)->mmio->SR2;
                if (!(sr1 & I2C_SR1_ADDR)) {
			return (ehw_sig(dev));
		}
		if (((i2cm) dev)->iadr_sz) {
#if I2CM_USE_TXE == 1
			((i2cm) dev)->mmio->CR2 |= I2C_CR2_ITBUFEN;
#endif
			((i2cm) dev)->mmio->DR = ((i2cm) dev)->iadr[((i2cm) dev)->iadr_idx++];
			((i2cm) dev)->iadr_sz--;
                        ((i2cm) dev)->st = EVN_WRST_NEXTB;
		} else {
			if (--((i2cm) dev)->sz) {
#if I2CM_USE_TXE == 1
				((i2cm) dev)->mmio->CR2 |= I2C_CR2_ITBUFEN;
#endif
				((i2cm) dev)->mmio->DR = *((i2cm) dev)->buf++;
				((i2cm) dev)->st = EVN_WRST_NEXTB;
			} else {
				((i2cm) dev)->mmio->DR = *((i2cm) dev)->buf++;
				((i2cm) dev)->st = EVN_WRST_STOP;
			}
		}
		break;
	case EVN_WRST_NEXTB :
#if I2CM_USE_TXE == 1
		if (!(sr1 & I2C_SR1_TXE)) {
#else
		if (!(sr1 & I2C_SR1_BTF)) {
#endif
			return (ehw_sig(dev));
		}
                if (((i2cm) dev)->iadr_sz) {
			((i2cm) dev)->mmio->DR = ((i2cm) dev)->iadr[((i2cm) dev)->iadr_idx++];
			((i2cm) dev)->iadr_sz--;
		} else {
			if (!--((i2cm) dev)->sz) {
#if I2CM_USE_TXE == 1
				((i2cm) dev)->mmio->CR2 &= ~I2C_CR2_ITBUFEN;
#endif
				((i2cm) dev)->mmio->DR = *((i2cm) dev)->buf++;
				((i2cm) dev)->st = EVN_WRST_STOP;
			} else {
				((i2cm) dev)->mmio->DR = *((i2cm) dev)->buf++;
			}
		}
		break;
	case EVN_WRST_STOP :
		if (!(sr1 & I2C_SR1_BTF)) {
			return (ehw_sig(dev));
		}
		((i2cm) dev)->mmio->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
		((i2cm) dev)->mmio->CR1 |= I2C_CR1_STOP;
		msg = 0;
		xQueueSendFromISR(((i2cm) dev)->sig_que, &msg, &tsk_wkn);
		break;
	case EVN_WRST_ADR10 :
		if (!(sr1 & I2C_SR1_ADD10)) {
			return (ehw_sig(dev));
		}
		((i2cm) dev)->mmio->DR = ((i2cm) dev)->adb2;
		((i2cm) dev)->st = EVN_WRST_ADR;
		break;
	}
	return (tsk_wkn);
}

/**
 * evn_iadr_stm
 */
static BaseType_t evn_iadr_stm(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	unsigned int sr1, sr2;

	sr1 = ((i2cm) dev)->mmio->SR1;
	switch (((i2cm) dev)->st) {
	case EVN_IADRST_START :
		if (!(sr1 & I2C_SR1_SB)) {
			return (ehw_sig(dev));
		}
		if (((i2cm) dev)->op == I2CM_READ_ADR7) {
			((i2cm) dev)->st = EVN_IADRST_ADR;
		} else {
			((i2cm) dev)->st = EVN_IADRST_ADR10;
		}
		((i2cm) dev)->mmio->DR = ((i2cm) dev)->adb1;
		break;
	case EVN_IADRST_ADR :
		sr2 = ((i2cm) dev)->mmio->SR2;
                if (!(sr1 & I2C_SR1_ADDR)) {
			return (ehw_sig(dev));
		}
		if (--((i2cm) dev)->iadr_sz) {
			((i2cm) dev)->mmio->DR = ((i2cm) dev)->iadr[((i2cm) dev)->iadr_idx++];
			((i2cm) dev)->st = EVN_IADRST_NEXTB;
		} else {
			((i2cm) dev)->mmio->DR = ((i2cm) dev)->iadr[((i2cm) dev)->iadr_idx++];
			((i2cm) dev)->mmio->CR1 |= I2C_CR1_START;
                        ((i2cm) dev)->mmio->DR = 0; // SR1_BTF bit clearing.
			if (((i2cm) dev)->op == I2CM_READ_ADR7) {
				reg_i2c_ev_isr_clbk(((i2cm) dev)->mmio, evn_rd_stm_a7);
				((i2cm) dev)->st = EVN_RDST_A7_START;
			} else {
				reg_i2c_ev_isr_clbk(((i2cm) dev)->mmio, evn_rd_stm_a10);
				((i2cm) dev)->st = EVN_RDST_A10_START;
			}
		}
		break;
	case EVN_IADRST_NEXTB :
		if (!(sr1 & I2C_SR1_BTF)) {
			return (ehw_sig(dev));
		}
		if (!--((i2cm) dev)->iadr_sz) {
			((i2cm) dev)->mmio->DR = ((i2cm) dev)->iadr[((i2cm) dev)->iadr_idx++];
                        ((i2cm) dev)->mmio->CR1 |= I2C_CR1_START;
                        ((i2cm) dev)->mmio->DR = 0; // SR1_BTF bit clearing.
			if (((i2cm) dev)->op == I2CM_READ_ADR7) {
				reg_i2c_ev_isr_clbk(((i2cm) dev)->mmio, evn_rd_stm_a7);
				((i2cm) dev)->st = EVN_RDST_A7_START;
			} else {
				reg_i2c_ev_isr_clbk(((i2cm) dev)->mmio, evn_rd_stm_a10);
				((i2cm) dev)->st = EVN_RDST_A10_START;
			}
		} else {
			((i2cm) dev)->mmio->DR = ((i2cm) dev)->iadr[((i2cm) dev)->iadr_idx++];
		}
		break;
	case EVN_IADRST_ADR10 :
		if (!(sr1 & I2C_SR1_ADD10)) {
			return (ehw_sig(dev));
		}
		((i2cm) dev)->mmio->DR = ((i2cm) dev)->adb2;
		((i2cm) dev)->st = EVN_IADRST_ADR;
		break;
	}
	return (tsk_wkn);
}

/**
 * evn_rd_stm_a7
 */
static BaseType_t evn_rd_stm_a7(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	unsigned int sr1;

	switch (((i2cm) dev)->st) {
	case EVN_RDST_A7_START :
		sr1 = ((i2cm) dev)->mmio->SR1;
		if (!(sr1 & I2C_SR1_SB)) {
			return (ehw_sig(dev));
		}
                ((i2cm) dev)->st = EVN_RDST_A7_ADR;
		((i2cm) dev)->mmio->DR = ((i2cm) dev)->adb1 | 1;
		break;
	case EVN_RDST_A7_ADR :
		if (!rd_start(dev)) {
			return (ehw_sig(dev));
		}
		break;
	}
	return (tsk_wkn);
}

/**
 * evn_rd_stm_a10
 */
static BaseType_t evn_rd_stm_a10(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	unsigned int sr1, sr2;

	switch (((i2cm) dev)->st) {
	case EVN_RDST_A10_START :
		sr1 = ((i2cm) dev)->mmio->SR1;
		if (!(sr1 & I2C_SR1_SB)) {
			return (ehw_sig(dev));
		}
                ((i2cm) dev)->st = EVN_RDST_A10_ADR10;
		((i2cm) dev)->mmio->DR = ((i2cm) dev)->adb1;
		break;
	case EVN_RDST_A10_ADR10 :
		sr1 = ((i2cm) dev)->mmio->SR1;
		if (!(sr1 & I2C_SR1_ADD10)) {
			return (ehw_sig(dev));
		}
		((i2cm) dev)->st = EVN_RDST_A10_ADR;
		((i2cm) dev)->mmio->DR = ((i2cm) dev)->adb2;
		break;
	case EVN_RDST_A10_ADR :
		sr1 = ((i2cm) dev)->mmio->SR1;
		sr2 = ((i2cm) dev)->mmio->SR2;
		if (!(sr1 & I2C_SR1_ADDR)) {
			return (ehw_sig(dev));
		}
                ((i2cm) dev)->st = EVN_RDST_A10_N_START;
		((i2cm) dev)->mmio->CR1 |= I2C_CR1_START;
		break;
	case EVN_RDST_A10_N_START :
		sr1 = ((i2cm) dev)->mmio->SR1;
		if (!(sr1 & I2C_SR1_SB)) {
			return (ehw_sig(dev));
		}
                ((i2cm) dev)->st = EVN_RDST_A10_N_HEAD;
		((i2cm) dev)->mmio->DR = ((i2cm) dev)->adb1 | 1;
		break;
	case EVN_RDST_A10_N_HEAD :
		if (!rd_start(dev)) {
			return (ehw_sig(dev));
		}
		break;
	}
	return (tsk_wkn);
}

/**
 * rd_start
 */
static boolean_t rd_start(void *dev)
{
	unsigned int sr1, sr2;

	if (((i2cm) dev)->sz == 1) {
		((i2cm) dev)->mmio->CR1 &= ~I2C_CR1_ACK;
		sr1 = ((i2cm) dev)->mmio->SR1;
		sr2 = ((i2cm) dev)->mmio->SR2;
                ((i2cm) dev)->mmio->CR2 |= I2C_CR2_ITBUFEN;
		if (!(sr1 & I2C_SR1_ADDR)) {
			return (FALSE);
		}
		((i2cm) dev)->mmio->CR1 |= I2C_CR1_STOP;
		reg_i2c_ev_isr_clbk(((i2cm) dev)->mmio, evn_rd_stm_b1);
	} else if (((i2cm) dev)->sz == 2) {
		((i2cm) dev)->mmio->CR1 |= I2C_CR1_POS;
		sr1 = ((i2cm) dev)->mmio->SR1;
		sr2 = ((i2cm) dev)->mmio->SR2;
		if (!(sr1 & I2C_SR1_ADDR)) {
			return (FALSE);
		}
		((i2cm) dev)->mmio->CR1 &= ~I2C_CR1_ACK;
		reg_i2c_ev_isr_clbk(((i2cm) dev)->mmio, evn_rd_stm_b2);
	} else {
		sr1 = ((i2cm) dev)->mmio->SR1;
		sr2 = ((i2cm) dev)->mmio->SR2;
                ((i2cm) dev)->mmio->CR2 |= I2C_CR2_ITBUFEN;
		if (!(sr1 & I2C_SR1_ADDR)) {
			return (FALSE);
		}
                if (((i2cm) dev)->sz == 3) {
			((i2cm) dev)->st = EVN_RDST_BN_3B;
		} else {
			((i2cm) dev)->st = EVN_RDST_BN_NB;
		}
		reg_i2c_ev_isr_clbk(((i2cm) dev)->mmio, evn_rd_stm_bn);
	}
	return (TRUE);
}

/**
 * evn_rd_stm_b1
 */
static BaseType_t evn_rd_stm_b1(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	unsigned int sr1;
	uint8_t msg;

	sr1 = ((i2cm) dev)->mmio->SR1;
	if (!(sr1 & I2C_SR1_RXNE)) {
		return (ehw_sig(dev));
	}
	*((i2cm) dev)->buf = ((i2cm) dev)->mmio->DR;
	((i2cm) dev)->sz--;
        msg = 0;
	((i2cm) dev)->mmio->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
	xQueueSendFromISR(((i2cm) dev)->sig_que, &msg, &tsk_wkn);
	return (tsk_wkn);
}

/**
 * evn_rd_stm_b2
 */
static BaseType_t evn_rd_stm_b2(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	unsigned int sr1;
	uint8_t msg;

	sr1 = ((i2cm) dev)->mmio->SR1;
	if (!(sr1 & I2C_SR1_BTF)) {
		return (ehw_sig(dev));
	}
	((i2cm) dev)->mmio->CR1 |= I2C_CR1_STOP;
	*((i2cm) dev)->buf++ = ((i2cm) dev)->mmio->DR;
	((i2cm) dev)->sz--;
	*((i2cm) dev)->buf = ((i2cm) dev)->mmio->DR;
	((i2cm) dev)->sz--;
        msg = 0;
	((i2cm) dev)->mmio->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
	xQueueSendFromISR(((i2cm) dev)->sig_que, &msg, &tsk_wkn);
	return (tsk_wkn);
}

/**
 * evn_rd_stm_bn
 */
static BaseType_t evn_rd_stm_bn(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	unsigned int sr1;
	uint8_t msg;

	switch (((i2cm) dev)->st) {
	case EVN_RDST_BN_NB :
		sr1 = ((i2cm) dev)->mmio->SR1;
		if (!(sr1 & I2C_SR1_RXNE)) {
			return (ehw_sig(dev));
		}
		*((i2cm) dev)->buf++ = ((i2cm) dev)->mmio->DR;
		if (--((i2cm) dev)->sz == 3) {
			((i2cm) dev)->st = EVN_RDST_BN_3B;
		}
		break;
	case EVN_RDST_BN_3B :
		sr1 = ((i2cm) dev)->mmio->SR1;
		if (!(sr1 & I2C_SR1_RXNE)) {
			return (ehw_sig(dev));
		}
                ((i2cm) dev)->mmio->CR2 &= ~I2C_CR2_ITBUFEN;
                ((i2cm) dev)->st = EVN_RDST_BN_BTF;
		break;
	case EVN_RDST_BN_BTF :
		sr1 = ((i2cm) dev)->mmio->SR1;
		if (!(sr1 & I2C_SR1_BTF)) {
			return (ehw_sig(dev));
		}
		((i2cm) dev)->mmio->CR2 |= I2C_CR2_ITBUFEN;
		((i2cm) dev)->mmio->CR1 &= ~I2C_CR1_ACK;
		*((i2cm) dev)->buf++ = ((i2cm) dev)->mmio->DR;
                ((i2cm) dev)->sz--;
		((i2cm) dev)->mmio->CR1 |= I2C_CR1_STOP;
		*((i2cm) dev)->buf++ = ((i2cm) dev)->mmio->DR;
                ((i2cm) dev)->sz--;
		((i2cm) dev)->st = EVN_RDST_BN_LAST;
		break;
	case EVN_RDST_BN_LAST :
		sr1 = ((i2cm) dev)->mmio->SR1;
		if (!(sr1 & I2C_SR1_RXNE)) {
			return (ehw_sig(dev));
		}
                *((i2cm) dev)->buf++ = ((i2cm) dev)->mmio->DR;
		((i2cm) dev)->sz--;
		msg = 0;
		((i2cm) dev)->mmio->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
		xQueueSendFromISR(((i2cm) dev)->sig_que, &msg, &tsk_wkn);
		break;
	}
	return (tsk_wkn);
}

/**
 * ehw_sig
 */
static BaseType_t ehw_sig(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	uint8_t msg = EHW;

	((i2cm) dev)->mmio->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
        ((i2cm) dev)->mmio->SR1 = 0;
	xQueueSendFromISR(((i2cm) dev)->sig_que, &msg, &tsk_wkn);
        return (tsk_wkn);
}

/**
 * err
 */
static BaseType_t err(void *dev)
{
	BaseType_t tsk_wkn = pdFALSE;
	uint8_t msg = EGEN;

	((i2cm) dev)->sr1_err = ((i2cm) dev)->mmio->SR1;
	((i2cm) dev)->mmio->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
        ((i2cm) dev)->mmio->SR1 = 0;
	if (((i2cm) dev)->sr1_err & I2C_SR1_AF) {
		((i2cm) dev)->mmio->CR1 |= I2C_CR1_STOP;
	}
	xQueueSendFromISR(((i2cm) dev)->sig_que, &msg, &tsk_wkn);
	return (tsk_wkn);
}

#if TERMOUT == 1
/**
 * log_i2c_errors
 */
void log_i2c_errors(i2cm dev)
{
	msg(INF, "i2cm.c: stm_tmo_err=%d stm_hw_err=%d bus_err=%d\n",
	    dev->stm_tmo_err, dev->stm_hw_err, dev->bus_err);
	msg(INF, "i2cm.c: ovr_err=%d msl_err=%d\n",
	    dev->ovr_err, dev->msl_err);
}

#if I2CM_DBG == 1
/**
 * log_i2c_stm
 */
void log_i2c_stm(i2cm dev)
{
	BaseType_t (*cfn)(void *);

	cfn = get_i2c_ev_isr_clbk(dev->mmio);
	if (cfn == evn_wr_stm) {
		msg(INF, "i2cm.c: STM->EVN_WRST_");
		switch (dev->st) {
		case EVN_WRST_START :
			msg(INF, "START\n");
			break;
		case EVN_WRST_ADR :
			msg(INF, "ADR\n");
			break;
		case EVN_WRST_ADR10 :
			msg(INF, "ADR10\n");
			break;
		case EVN_WRST_NEXTB :
			msg(INF, "NEXTB\n");
			break;
		case EVN_WRST_STOP :
			msg(INF, "STOP\n");
			break;
		default :
			msg(INF, "UNK\n");
			break;
		}
	} else if (cfn == evn_iadr_stm) {
		msg(INF, "i2cm.c: STM->EVN_IADRST_");
		switch (dev->st) {
		case EVN_IADRST_START :
			msg(INF, "START\n");
			break;
		case EVN_IADRST_ADR :
			msg(INF, "ADR\n");
			break;
		case EVN_IADRST_ADR10 :
			msg(INF, "ADR10\n");
			break;
		case EVN_IADRST_NEXTB :
			msg(INF, "NEXTB\n");
			break;
		default :
			msg(INF, "UNK\n");
			break;
		}
	} else if (cfn == evn_rd_stm_a7) {
		msg(INF, "i2cm.c: STM->EVN_RDST_A7_");
		switch (dev->st) {
		case EVN_RDST_A7_START :
			msg(INF, "START\n");
			break;
		case EVN_RDST_A7_ADR :
			msg(INF, "ADR\n");
			break;
		default :
			msg(INF, "UNK\n");
			break;
		}
	} else if (cfn == evn_rd_stm_a10) {
		msg(INF, "i2cm.c: STM->EVN_RDST_A10_");
		switch (dev->st) {
		case EVN_RDST_A10_START :
			msg(INF, "START\n");
			break;
		case EVN_RDST_A10_ADR10 :
			msg(INF, "ADR10\n");
			break;
		case EVN_RDST_A10_ADR :
			msg(INF, "ADR\n");
			break;
		case EVN_RDST_A10_N_START :
			msg(INF, "N_START\n");
			break;
		case EVN_RDST_A10_N_HEAD :
			msg(INF, "N_HEAD\n");
			break;
		default :
			msg(INF, "UNK\n");
			break;
		}
	} else if (cfn == evn_rd_stm_b1) {
		msg(INF, "i2cm.c: STM->EVN_RD_STM_B1\n");
	} else if (cfn == evn_rd_stm_b2) {
		msg(INF, "i2cm.c: STM->EVN_RD_STM_B2\n");
	} else if (cfn == evn_rd_stm_bn) {
		msg(INF, "i2cm.c: STM->EVN_RDST_BN_");
		switch (dev->st) {
		case EVN_RDST_BN_3B :
			msg(INF, "3B\n");
			break;
		case EVN_RDST_BN_NB :
			msg(INF, "NB\n");
			break;
		case EVN_RDST_BN_BTF :
			msg(INF, "BTF\n");
			break;
		case EVN_RDST_BN_LAST :
			msg(INF, "LAST\n");
			break;
		default :
			msg(INF, "UNK\n");
			break;
		}
	} else {
		msg(INF, "i2cm.c: STM->UNKNOWN_STM\n");
	}
}
#endif

#endif

#endif
