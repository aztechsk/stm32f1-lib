/*
 * i2cm.h
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

#ifndef I2CM_H
#define I2CM_H

#ifndef I2CM
 #define I2CM 0
#endif

#if I2CM == 1

enum i2c_mode {
	I2C_MODE_STD,
        I2C_MODE_FAST
};

enum i2c_fm_duty {
	I2C_FM_DUTY_2,   // tlow/thigh = 2.
        I2C_FM_DUTY_16_2 // tlow/thigh = 16/9.
};

enum i2c_pins_mode {
	I2C_PINS_PERIPHERAL, // Controlled by I2C.
	I2C_PINS_GPIO,       // I2C disabled.
	I2C_PINS_OPEN_DRAIN, // Controlled by GPIO (open drain output HIGH).
	I2C_PIN_SCL_HIGH,    // Open drain output HIGH.
	I2C_PIN_SCL_LOW,     // Open drain output LOW.
	I2C_PIN_SDA_HIGH,    // Open drain output HIGH.
	I2C_PIN_SDA_LOW      // Open drain output LOW.
};

enum i2cm_oper {
	I2CM_READ_ADR7,
        I2CM_READ_ADR10,
        I2CM_WRITE_ADR7,
        I2CM_WRITE_ADR10,
        I2CM_READ_ADR7_IADR1, // Internal device address 1 byte.
        I2CM_READ_ADR7_IADR2, // Internal device address 2 bytes.
        I2CM_READ_ADR10_IADR1,
        I2CM_READ_ADR10_IADR2,
        I2CM_WRITE_ADR7_IADR1,
        I2CM_WRITE_ADR7_IADR2,
        I2CM_WRITE_ADR10_IADR1,
        I2CM_WRITE_ADR10_IADR2
};

typedef struct i2cm_dsc *i2cm;

struct i2cm_dsc {
	I2C_TypeDef *mmio; // <SetIt>
	int speed; // <SetIt>
	enum i2c_mode mode; // <SetIt>
        enum i2c_fm_duty duty; // <SetIt>
        void (*conf_pins)(enum i2c_pins_mode); // <SetIt>
        boolean_t use_mtx; // <SetIt>.
	unsigned int apb_bmp;
	int irqn_ev;
	int irqn_er;
        unsigned int reg_cr2;
        unsigned int reg_trise;
        unsigned int reg_ccr;
	uint8_t adb1;
	uint8_t adb2;
        enum i2cm_oper op;
        QueueHandle_t sig_que;
        SemaphoreHandle_t mtx;
	int sz;
        uint8_t *buf;
	int st;
	int iadr_sz;
	int iadr_idx;
	uint8_t iadr[4];
        uint16_t sr1_err;
	int stm_tmo_err;
	int stm_hw_err;
	int bus_err;
	int ovr_err;
        int msl_err;
};

/**
 * init_i2cm
 *
 * Configures I2C instance in master (controller) mode.
 *
 * @dev: I2C instance.
 */
void init_i2cm(i2cm dev);

/**
 * lock_i2c_bus
 *
 * Lock I2C bus.
 *
 * @dev: I2C instance.
 */
void lock_i2c_bus(i2cm dev);

/**
 * unlock_i2c_bus
 *
 * Unlock I2C bus.
 *
 * @dev: I2C instance.
 */
void unlock_i2c_bus(i2cm dev);

/**
 * reset_i2c_bus
 *
 * Sends reset sequence to I2C bus.
 *
 * @dev: I2C instance.
 */
void reset_i2c_bus(i2cm dev);

/**
 * enable_i2cm
 *
 * Enables I2C instance (reverts disable_i2cm() function effects).
 *
 * @dev: I2C instance.
 */
void enable_i2cm(i2cm dev);

/**
 * disable_i2cm
 *
 * Disables I2C instance (switches I2C block off).
 *
 * @dev: I2C instance.
 */
void disable_i2cm(i2cm dev);

/**
 * i2cm_io
 *
 * Reads / writes bytes on i2c slave device.
 * Caller task is blocked until operation terminated.
 *
 * @dev: I2C instance.
 * @op: Read / write I2C operation (enum i2cm_oper).
 * @adr: Slave address (7 or 10 bit integer number).
 *   Number is translated to raw binary address in function.
 * @p_buf: Pointer to memory data buffer.
 * @size: Bytes count.
 * @va1: Slave memory internal address for i2cm_oper *_IADR modes.
 *
 * Returns: 0 - success; -ENACK - slave NACK; -EBUSY - I2C bus busy;
 *   -EACC - I2C bus arbitration lost; -EHW - hardware in unexpected state.
 *
 * I2C peripheral must be reset after EHW erorr condition or longer
 * EBUSY condition is detected.
 */
int i2cm_io(i2cm dev, enum i2cm_oper op, int adr, uint8_t *p_buf, int size, ...);

#if TERMOUT == 1
/**
 * log_i2c_errors
 */
void log_i2c_errors(i2cm dev);

#if I2CM_DBG == 1
/**
 * log_i2c_stm
 */
void log_i2c_stm(i2cm dev);
#endif

#endif

#endif

#endif
