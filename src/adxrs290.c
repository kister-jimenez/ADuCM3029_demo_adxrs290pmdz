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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdlib.h>
#include "error.h"
#include "util.h"
#include "adxrs290.h"

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Read device register.
 * @param dev - Device handler.
 * @param address - Register address.
 * @param data - Pointer to the register value container.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adxrs290_reg_read(struct adxrs290_dev *dev, uint8_t address,
			  uint8_t *data)
{
	uint8_t buff[] = {address | 0x80, 0};

	if(spi_write_and_read(dev->spi_desc, buff, 2) != SUCCESS)
		return FAILURE;

	*data = (uint8_t)buff[1];
	return SUCCESS;
}

/**
 * @brief Write device register.
 * @param dev - Device handler.
 * @param address - Register address.
 * @param data - New register value.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adxrs290_reg_write(struct adxrs290_dev *dev, uint8_t address,
			   uint8_t data)
{
	uint8_t buff[] = {address & 0x7F, data};

	if(spi_write_and_read(dev->spi_desc, buff, 2) != SUCCESS)
		return FAILURE;
	return SUCCESS;
}

/**
 * @brief Do a read and write of a register to update only part of a register.
 * @param dev - Device handler.
 * @param address - Address of the register.
 * @param data - Value to be written to the device.
 * @param mask - Mask of the bit field to update.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adxrs290_reg_write_mask(struct adxrs290_dev *dev, uint8_t address,
				uint8_t data, uint8_t mask)
{
	int32_t ret;
	uint16_t reg_val;
	uint32_t bit_pos;

	ret = adxrs290_reg_read(dev, address, &reg_val);
	if (ret != SUCCESS)
		return FAILURE;
	reg_val &= ~mask;
	bit_pos = find_first_set_bit((uint32_t)mask);
	reg_val |= (data << bit_pos) & mask;

	return adxrs290_reg_write(dev, address, reg_val);
}

/**
 * @brief Get the low-pass filter pole location.
 * @param dev - Device handler.
 * @param lpf - Pointer to Low-pass pole location container.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adxrs290_get_lpf(struct adxrs290_dev *dev, enum adxrs290_lpf *lpf)
{
	uint8_t data;
	if(adxrs290_reg_read(dev, ADXRS290_REG_FILTER, &data) != SUCCESS)
		return FAILURE;
	*lpf = ADXRS290_LPF(data);
	return SUCCESS;
}

/**
 * @brief Set the low-pass filter pole location.
 * @param dev - Device handler.
 * @param lpf - Low-pass pole location.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adxrs290_set_lpf(struct adxrs290_dev *dev, enum adxrs290_lpf lpf)
{
	return adxrs290_reg_write_mask(dev, ADXRS290_REG_FILTER,
			lpf, ADXRS290_LPF_MASK);
}

/**
 * @brief Get the high-pass filter pole location.
 * @param dev - Device handler.
 * @param hpf - Pointer to high-pass pole location container.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adxrs290_get_hpf(struct adxrs290_dev *dev, enum adxrs290_hpf *hpf)
{
	uint8_t data;
	if(adxrs290_reg_read(dev, ADXRS290_REG_FILTER, &data) != SUCCESS)
		return FAILURE;
	*hpf = ADXRS290_HPF(data);
	return SUCCESS;
}

/**
 * @brief Set the low-pass filter pole location.
 * @param dev - Device handler.
 * @param hpf - High-pass pole location.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adxrs290_set_hpf(struct adxrs290_dev *dev, enum adxrs290_hpf hpf)
{
	return adxrs290_reg_write_mask(dev, ADXRS290_REG_FILTER,
			hpf, ADXRS290_HPF_MASK);
}

/**
 * Initialize the device.
 * @param device - The device structure.
 * @param init_param - The structure that contains the device initial
 *		       parameters.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t adxrs290_init(struct adxrs290_dev **device,
		     struct adxrs290_init_param init_param)
{
	struct adxrs290_dev *dev;
	int32_t ret = 0;

	dev = (struct adxrs290_dev *)malloc(sizeof(*dev));
	if (!dev)
		goto error;

	ret = spi_init(&dev->spi_desc, &init_param.spi_init);
	if (!ret)
		goto error;

	if (adxrs290_get_register_value(dev, ADXRS290_REG_DEV_ID)
			!= ADXRS290_DEV_ID)
		goto error;

	*device = dev;

	return ret;

error:
	printf("adxrs290 initialization error (%d)\n", ret);
	free(dev);
	mdelay(1000);
	return ret;
}

/**
 * @brief Free memory allocated by adxrs290_setup().
 * @param dev - Device handler.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adxl345_remove(struct adxrs290_dev *dev)
{
	int32_t ret;

	ret = spi_remove(dev->spi_desc);

	free(dev);

	return ret;
}


