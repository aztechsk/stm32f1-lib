/*
 * modbm.h
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

#ifndef MODBM_H
#define MODBM_H

#ifndef MODBM
 #define MODBM 0
#endif

#if MODBM == 1

// MODBUS TABLES - DATA MODEL
//---------------------------------------------
// Primary table     | Object type            |
//---------------------------------------------
// Discretes Inputs  | Single bit read-only   |
// Coils             | Single bit read-write  |
// Input Registers   | 16-bit word read-only  |
// Holding Registers | 16-bit word read-write |
//---------------------------------------------

// ADDRESSING.
// In MODBUS data model each element within data table is numbered from 1 to n.
// In MODBUS PDU each data is addressed from 0 to 65535.

enum modbm_instance {
	MODBM_USART1,
	MODBM_USART2,
        MODBM_USART3
};

enum modbm_parity {
	MODBM_PARITY_NO,
	MODBM_PARITY_EVEN,
        MODBM_PARITY_ODD
};

enum modbm_pins_mode {
	MODBM_PINS_PERIPHERAL,
	MODBM_PINS_GPIO
};

struct modbm_stats {
	int unexp_intr;
	int par_lerr;
        int fra_lerr;
        int noi_lerr;
	int ovr_lerr;
	int short_msg;
        int crc_err;
        int bad_adr;
	int fmt_err;
	int idle_err;
};

typedef struct modbm_dsc *modbm;

struct modbm_dsc {
	enum modbm_instance instance; // <SetIt>
        int baudrate; // <SetIt>
        enum modbm_parity parity; // <SetIt>
        void (*conf_pins)(enum modbm_pins_mode); // <SetIt>
	void (*de_pin_ctl)(boolean_t lev); // <SetIt>
	void (*wait_idle)(void); // <SetIt> If NULL, timing of transactions must be provided externally
	int buf_len; // <SetIt>, max 256
        QueueHandle_t sig_que;
	USART_TypeDef *mmio;
	int apb_inst;
	unsigned int apb_bmp;
	int irqn;
	unsigned int reg_brr;
        unsigned int reg_cr1;
        unsigned int reg_cr2;
        unsigned int reg_cr3;
	uint8_t *buf;
        uint8_t *p;
	int s_sz;
        int r_sz;
        int r_cnt;
        struct modbm_stats stats;
};

// Function parameter structure should be dynamically allocated with flexible
// array member 'regs' according to value in 'reg_cnt' member.
struct modbm_reg_io_req {
	// Starting register number in MODBUS data model (from 1 to n).
	int reg_start;
        // Number of registers.
	int reg_cnt;
	// Timeout for slave answer in miliseconds.
        int tmo_ms;
	// Exception code.
	int except;
	// Registers array.
	uint16_t regs[];
};

// Function parameter structure should be dynamically allocated with flexible
// array member 'bits' according to value in 'bit_cnt' member.
struct modbm_bit_io_req {
	// Starting bit number in MODBUS data model (from 1 to n).
	int bit_start;
        // Number of bits.
	int bit_cnt;
	// Timeout for slave answer in miliseconds.
        int tmo_ms;
	// Exception code.
	int except;
	// Bits array.
	// Bit number 1 is placed in uint8_t as LSB and bit number 8 as MSB.
	uint8_t bits[];
};

enum modbm_reg_table {
	MODBM_INPUT_REG_TABLE,
        MODBM_HOLDING_REG_TABLE
};

enum modbm_bit_table {
	MODBM_DISC_INPUT_TABLE,
        MODBM_COILS_TABLE
};

/**
 * init_modbm
 *
 * Configures MODBM instance.
 *
 * @dev: MODBM instance.
 */
void init_modbm(modbm dev);

/**
 * modbm_read_registers
 *
 * Reads one or more registers from HOLDING or INPUT registers table.
 * Caller task is blocked until requested data is received or timeout is expired.
 *
 * @dev: MODBM instance.
 * @adr: Slave address.
 * @table: Registers table selection (enum modbm_reg_table).
 * @req: MODBUS request structure. Member 'reg_start' 'reg_cnt' and 'tmo_ms'
 *   must be set. Readed registers will be stored in 'regs' array member.
 *
 * Returns: 0 - success; -ETMO - slave not answer; -ENACK - slave reports error
 *   (MODBUS exception code is returned in 'req->except' member);
 *   -EGEN - generic error.
 */
int modbm_read_registers(modbm dev, int adr, enum modbm_reg_table table, struct modbm_reg_io_req *req);

/**
 * modbm_write_registers
 *
 * Writes one or more registers to HOLDING registers table.
 * Caller task is blocked until slave answer or timeout is expired.
 *
 * @dev: MODBM instance.
 * @adr: Slave address.
 * @req: MODBUS request structure. Member 'reg_start' 'reg_cnt' and 'tmo_ms'
 *   must be set. Registers for write must be stored in 'regs' array member.
 *
 * Returns: 0 - success; -ETMO - slave not answer; -ENACK - slave reports error
 *   (MODBUS exception code is returned in 'req->except' member);
 *   -EGEN - generic error.
 */
int modbm_write_registers(modbm dev, int adr, struct modbm_reg_io_req *req);

/**
 * modbm_write_register
 *
 * Writes one register to HOLDING registers table.
 * Caller task is blocked until slave answer or timeout is expired.
 *
 * @dev: MODBM instance.
 * @adr: Slave address.
 * @req: MODBUS request structure. Member 'reg_start' and 'tmo_ms' must be set.
 *   Register for write must be stored in 'regs[0]' array member.
 *
 * Returns: 0 - success; -ETMO - slave not answer; -ENACK - slave reports error
 *   (MODBUS exception code is returned in 'req->except' member);
 *   -EGEN - generic error.
 */
int modbm_write_register(modbm dev, int adr, struct modbm_reg_io_req *req);

/**
 * modbm_read_bits
 *
 * Reads one or more bits from DISCRETES INPUTS or COILS bits table.
 * Caller task is blocked until requested data is received or timeout is expired.
 *
 * @dev: MODBM instance.
 * @adr: Slave address.
 * @table: Bits table selection (enum modbm_bit_table).
 * @req: MODBUS request structure. Member 'bit_start' 'bit_cnt' and 'tmo_ms' must
 *   be set. Readed bits will be stored in 'bits' array member.
 *
 * Returns: 0 - success; -ETMO - slave not answer; -ENACK - slave reports error
 *   (MODBUS exception code is returned in 'req->except' member);
 *   -EGEN - generic error.
 */
int modbm_read_bits(modbm dev, int adr, enum modbm_bit_table table, struct modbm_bit_io_req *req);

/**
 * modbm_write_bits
 *
 * Writes one or more bits to COILS bits table.
 * Caller task is blocked until slave answer or timeout is expired.
 *
 * @dev: MODBM instance.
 * @adr: Slave address.
 * @req: MODBUS request structure. Member 'bit_start' 'bit_cnt' and 'tmo_ms' must
 *   be set. Bits for write must be stored in 'bits' array member.
 *
 * Returns: 0 - success; -ETMO - slave not answer; -ENACK - slave reports error
 *   (MODBUS exception code is returned in 'req->except' member);
 *   -EGEN - generic error.
 */
int modbm_write_bits(modbm dev, int adr, struct modbm_bit_io_req *req);

/**
 * modbm_write_bit
 *
 * Writes one bit to COILS bits table.
 * Caller task is blocked until slave answer or timeout is expired.
 *
 * @dev: MODBM instance.
 * @adr: Slave address.
 * @req: MODBUS request structure. Member 'bit_start' and 'tmo_ms' must be set.
 *   Bit for write must be stored in lsb 'bits[0]' array member.
 *
 * Returns: 0 - success; -ETMO - slave not answer; -ENACK - slave reports error
 *   (MODBUS exception code is returned in 'req->except' member);
 *   -EGEN - generic error.
 */
int modbm_write_bit(modbm dev, int adr, struct modbm_bit_io_req *req);

#if TERMOUT == 1
/**
 * log_modbm_errors
 */
void log_modbm_errors(modbm dev);
#endif

#endif

#endif
