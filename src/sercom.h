/*
 * sercom.h
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

#ifndef SERCOM_H
#define SERCOM_H

#ifndef SERCOM_RX_CHAR
 #define SERCOM_RX_CHAR 0
#endif

#ifndef SERCOM_HDLC
 #define SERCOM_HDLC 0
#endif

#if SERCOM_HDLC == 1
struct hdlc_mesg {
	int sz;
	uint8_t *pld;
};

struct hdlc_stats {
        int par_lerr;      // HDLC
	int fra_lerr;      // HDLC
	int noi_lerr;      // HDLC
	int ovr_lerr;      // HDLC
        int no_f1_perr;    // HDLC
        int bf_ov_perr;    // HDLC
        int es_sq_perr;    // HDLC
	int syn_f1_perr;   // HDLC
};
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
enum sercom_instance {
	SERCOM_USART1,
	SERCOM_USART2,
        SERCOM_USART3
};

enum sercom_mode {
	SERCOM_RX_CHAR_MODE,
        SERCOM_HDLC_MODE
};

enum sercom_parity {
	SERCOM_PARITY_NO,
	SERCOM_PARITY_EVEN,
        SERCOM_PARITY_ODD
};

enum sercom_stop_bit {
	SERCOM_1_STOP_BIT,
	SERCOM_2_STOP_BITS
};

#if SERCOM_RX_CHAR == 1
enum sercom_char_size {
        SERCOM_CHAR_8_BITS,
        SERCOM_CHAR_9_BITS
};
#endif

enum sercom_pins_mode {
	SERCOM_PINS_PERIPHERAL,
	SERCOM_PINS_GPIO
};

typedef struct sercom_dsc *sercom;

struct sercom_dsc {
	enum sercom_instance instance; // <SetIt> [SERCOM_RX_CHAR_MODE, SERCOM_HDLC]
	int baudrate; // <SetIt> [SERCOM_RX_CHAR_MODE, SERCOM_HDLC]
        enum sercom_parity parity; // <SetIt> [SERCOM_RX_CHAR_MODE, SERCOM_HDLC]
        enum sercom_stop_bit stop_bit; // <SetIt> [SERCOM_RX_CHAR_MODE, SERCOM_HDLC]
#if SERCOM_RX_CHAR == 1
        enum sercom_char_size char_size; // <SetIt> [SERCOM_RX_CHAR_MODE]
        int rx_que_size; // <SetIt> [SERCOM_RX_CHAR_MODE]
#endif
        void (*conf_pins)(enum sercom_pins_mode); // <SetIt> [SERCOM_RX_CHAR_MODE, SERCOM_HDLC]
#if SERCOM_HDLC == 1
	int hdlc_bf_sz; // <SetIt> [SERCOM_HDLC]
	int hdlc_flag; // <SetIt> [SERCOM_HDLC]
	int hdlc_esc; // <SetIt> [SERCOM_HDLC]
	int hdlc_mod; // <SetIt> [SERCOM_HDLC]
	struct hdlc_mesg hdlc_mesg;
	struct hdlc_stats hdlc_stats;
        int rcv_st;
        QueueHandle_t hdlc_rx_sig;
#endif
	USART_TypeDef *mmio;
	int apb_inst;
	unsigned int apb_bmp;
	int irqn;
#if SERCOM_RX_CHAR == 1
	boolean_t b16;
	int size;
        void *p_buf;
#endif
        enum sercom_mode mode;
        TaskHandle_t c_tsk;
#if SERCOM_RX_CHAR == 1
        QueueHandle_t rx_que;
#endif
	boolean_t sr_pe;
	unsigned int reg_brr;
        unsigned int reg_cr1;
        unsigned int reg_cr2;
        unsigned int reg_cr3;
};
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
/**
 * init_sercom
 *
 * Configures SERCOM instance as UART in requested mode.
 *
 * @dev: SERCOM instance.
 * @m: SERCOM mode (enum sercom_mode).
 */
void init_sercom(sercom dev, enum sercom_mode m);
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
/**
 * enable_sercom
 *
 * Enables SERCOM (reverts disable_sercom() function effects).
 *
 * @dev: SERCOM instance.
 */
void enable_sercom(void *dev);
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
/**
 * disable_sercom
 *
 * Disables SERCOM (switches SERCOM block off).
 *
 * @dev: SERCOM instance.
 */
void disable_sercom(void *dev);
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
/**
 * sercom_tx_buff
 *
 * Transmits data buffer via SERCOM instance.
 * Caller task is blocked during sending data.
 *
 * @dev: SERCOM instance.
 * @p_buf: Pointer to data buffer of uint8_t or uint16_t type units
 *   (depends on sercom_parity & sercom_char_size).
 * @size: Number of data units to send.
 *
 * Returns: 0 - success; -EDMA - dma error.
 */
int sercom_tx_buff(void *dev, void *p_buf, int size);
#endif

#if SERCOM_RX_CHAR == 1
/**
 * sercom_rx_char
 *
 * Receives char via SERCOM instance.
 * Caller task is blocked until char is received, timeout is expired
 * or INTR event detected.
 *
 * @dev: SERCOM instance.
 * @p_char: Pointer to uint8_t or uint16_t memory for store received char
 *   (depends on sercom_parity & sercom_char_size).
 * @tmo: Timeout in tick periods.
 *
 * Returns: 0 - success; -ETMO - no data received in tmo time;
 *   -ERCV - serial line error; -EINTR - receiver interrupted.
 *   In case of ERCV, error bitmap is stored via p_char.
 *   Bit0 - Parity error.
 *   Bit1 - Framing error.
 *   Bit2 - Noise error.
 *   Bit3 - Overrun error.
 */
int sercom_rx_char(void *dev, void *p_char, TickType_t tmo);
#endif

#if SERCOM_HDLC == 1
/**
 * sercom_tx_hdlc_mesg
 *
 * Creates HDLC message from payload bytes and transmits it through
 * SERCOM instance.
 * Caller task is blocked during sending message.
 *
 * @dev: SERCOM instance.
 * @p_pld: Pointer to payload data.
 * @size: Size of payload data.
 *
 * Returns: 0 - success; -EBFOV - construction of HDLC message failed;
 *   -EDMA - dma error.
 */
int sercom_tx_hdlc_mesg(sercom dev, uint8_t *p_pld, int size);
#endif

#if SERCOM_HDLC == 1
/**
 * sercom_rx_hdlc_mesg
 *
 * Receives HDLC formated raw message through SERCOM instance.
 * Caller task is blocked until message is received or timeout is expired.
 *
 * @dev: SERCOM instance.
 * @tmo: Timeout in tick periods.
 *
 * Returns: struct hdlc_mesg * - message was successfully received;
 *   NULL - timeout.
 */
struct hdlc_mesg *sercom_rx_hdlc_mesg(sercom dev, TickType_t tmo);
#endif

#if SERCOM_RX_CHAR == 1 || SERCOM_HDLC == 1
/**
 * sercom_intr_rx
 *
 * Sends INTR event to SERCOM receiver.
 *
 * @dev: SERCOM instance.
 *
 * Returns: TRUE - successfully sent; FALSE - rx queue full.
 */
boolean_t sercom_intr_rx(void *dev);
#endif

#if SERCOM_RX_CHAR == 1
/**
 * sercom_flush_rx
 *
 * Disables receiver and flush receive buffers.
 *
 * @dev: SERCOM instance.
 */
void sercom_flush_rx(void *dev);
#endif

#endif
