/***************************************************************************//**
 *   @file   adxrs290.c
 *   @author Kister Genesis Jimenez (kister.jimenez@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef ADXRS290_H_
#define ADXRS290_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdbool.h>
#include "spi.h"
#include "gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/* ADXRS290 ID */
#define ADXRS290_ADI_ID							0xAD
#define ADXRS290_MEMS_ID						0x1D
#define ADXRS290_DEV_ID							0x92

#define ADXRS290_REG_ADI_ID						0x00
#define ADXRS290_REG_MEMS_ID					0x01
#define ADXRS290_REG_DEV_ID						0x02
#define ADXRS290_REG_REV_ID						0x03
#define ADXRS290_REG_SN0						0x04
#define ADXRS290_REG_SN1						0x05
#define ADXRS290_REG_SN2						0x06
#define ADXRS290_REG_SN3						0x07
#define ADXRS290_REG_DATAX0						0x08
#define ADXRS290_REG_DATAX1						0x09
#define ADXRS290_REG_DATAY0						0x0A
#define ADXRS290_REG_DATAY1						0x0B
#define ADXRS290_REG_TEMP0						0x0C
#define ADXRS290_REG_TEMP1						0x0D
//RESERVED
//RESERVED
#define ADXRS290_REG_POWER_CTL					0x10
#define ADXRS290_REG_FILTER						0x11
#define ADXRS290_REG_DATA_READY					0x12

/* ADXRS290_POWER_CTL */
#define ADXRS290_TSM							BIT(0)
#define ADXRS290_MEASUREMENT					BIT(1)

/* ADXRS290_Filter */
#define ADXRS290_LPF_MASK						0x07
#define ADXRS290_LPF(x)							(x) & ADXRS290_LPF_MASK
#define ADXRS290_HPF_MASK						0xF0
#define ADXRS290_HPF(x)							((x) & ADXRS290_LPF_MASK )>4
/* ADXRS290_DATA_READY */
#define ADXRS290_SYNC							0x03


/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/**
 * @enum adxrs290_mode
 * @brief Mode of the adxrs290
 */
enum adxrs290_mode {
	ADXRS290_MODE_STANDBY,
	ADXRS290_MODE_MEASUREMENT,
};

/**
* @enum adxrs290_channel
* @brief Channel of teh adxrs290 data rate
*/
enum adxrs290_channel {
	ADXRS290_CHANNEL_X,
	ADXRS290_CHANNEL_Y
 };
/**
 * @enum adxrs290_lpf
 * @brief Low-Pass filter pole location
 */
enum adxrs290_lpf {
	ADXRS290_LPF_480HZ,
	ADXRS290_LPF_320HZ,
	ADXRS290_LPF_160HZ,
	ADXRS290_LPF_80HZ,
	ADXRS290_LPF_56HZ6,
	ADXRS290_LPF_40HZ,
	ADXRS290_LPF_28HZ3,
	ADXRS290_LPF_20HZ
};

 /**
  * @enum adxrs290_hpf
  * @brief High-Pass filter pole location
  */
 enum adxrs290_hpf {
	ADXRS290_HPF_ALL_PASS,
 	ADXRS290_HPF_0HZ011,
	ADXRS290_HPF_0HZ022,
	ADXRS290_HPF_0HZ044,
	ADXRS290_HPF_0HZ087,
	ADXRS290_HPF_0HZ175,
	ADXRS290_HPF_0HZ350,
	ADXRS290_HPF_0HZ700,
	ADXRS290_HPF_1HZ400,
	ADXRS290_HPF_2HZ800,
	ADXRS290_HPF_11HZ30
 };

/**
 * @struct adxrs290_init_param
 * @brief Device driver initialization structure
 */
struct adxrs290_init_param {
	/** SPI Initialization structure. */
	spi_init_param	spi_init;
	/** GPIO 0 initialization */
	struct gpio_init_param sync;
	enum adxrs290_mode mode;
	enum adxrs290_lpf lpf;
	enum adxrs290_hpf hpf;

};

/**
 * @struct adxrs290_dev
 * @brief Device driver handler
 */
struct adxrs290_dev {
	/** SPI handler */
	struct spi_desc *spi_desc;
	/** Sync handler */
	struct gpio_desc *sync;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/** Read device register. */
int32_t adxrs290_reg_read(struct adxrs290_dev *dev, uint8_t address,
			  uint8_t *data);

/** Write device register. */
int32_t adxrs290_reg_write(struct adxrs290_dev *dev, uint8_t address,
			   uint8_t data);

/** Read device register multi-byte.*/
int32_t adxrs290_reg_readN(struct adxrs290_dev *dev, uint8_t start_address,
			  uint8_t *data, uint8_t num_regs);

/** Get the low-pass filter pole location. */
int32_t adxrs290_get_lpf(struct adxrs290_dev *dev, enum adxrs290_lpf *lpf);

/** Set the low-pass filter pole location. */
int32_t adxrs290_set_lpf(struct adxrs290_dev *dev, enum adxrs290_lpf lpf);

/** Get the high-pass filter pole location. */
int32_t adxrs290_get_hpf(struct adxrs290_dev *dev, enum adxrs290_hpf *hpf);

/** Set the high-pass filter pole location. */
int32_t adxrs290_set_hpf(struct adxrs290_dev *dev, enum adxrs290_hpf hpf);

/** Set enable irq and set callback */
int32_t adxrs290_register_callback(struct adxrs290_dev *dev, void *pf_callback);

/** Set mode */
int32_t adxrs290_set_op_mode(struct adxrs290_dev *dev, enum adxrs290_mode mode);

/** Read Temperature data */
int32_t adxrs290_get_temp_data(struct adxrs290_dev *dev, int16_t *temp);

/** Read X or Y angular velocity data */
int32_t adxrs290_get_rate_data(struct adxrs290_dev *dev,
		enum adxrs290_channel ch, int16_t *rate);

/** Read both the X and Y angular rate. */
int32_t adxrs290_get_rate_dataXY(struct adxrs290_dev *dev,
		int16_t *rateX, int16_t *rateY);

/*! Init. the comm. peripheral and checks if the ADXRS290 part is present. */
int32_t adxrs290_init(struct adxrs290_dev **device,
		     const struct adxrs290_init_param *init_param);

/*! Free the resources allocated by adxrs290_init(). */
int32_t adxrs290_remove(struct adxrs290_dev *dev);


#endif /* ADXRS290_H_ */
