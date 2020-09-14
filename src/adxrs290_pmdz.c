/***************************************************************************//**
*   @file   adxrs290_pmdz.c
*   @brief  Application source code.
*   @author Kister Genesis Jimenez (kister.jimenez@analog.com)
********************************************************************************
* Copyright 2019(c) Analog Devices, Inc.
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
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
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

#include "adxrs290_pmdz.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "error.h"
#include "timer.h"
#include "delay.h"

/******************************************************************************/
/************************** Variable Definitions ******************************/
/******************************************************************************/

/* Command vector */
static cmd_func adxrs290_fnc_ptr[] = {
	(cmd_func)adxrs290_pmdz_help,
	(cmd_func)adxrs290_pmdz_set_lpf,
	(cmd_func)adxrs290_pmdz_set_hpf,
	(cmd_func)adxrs290_pmdz_read_data,
	(cmd_func)adxrs290_pmdz_prod_test,
	NULL
};

/* Command call vector */
static char *adxrs290_fnc_calls[] = {
	"help",
	"h",
	"set_lpf ",
	"sl ",
	"set_hpf ",
	"sh ",
	"read_rate ",
	"r ",
	"prod_test",
	"t",
	""
};

/* Command size vector */
static uint8_t adxrs290_fnc_call_size[] = {
	5, 2, 8, 3, 8, 3, 10, 2, 10, 2, 1
};

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * Help command helper function. Display help function prompt.
 *
 * @param [in] dev	 	 	 - The device structure.
 * @param [in] short_command - True to display the long command prompt,
 *                             false to display the short command prompt.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
static int32_t adxrs290_pmdz_help_prompt(struct adxrs290_pmdz_dev *dev,
					bool short_command)
{
	int32_t ret;

	if (!short_command) {
		ret = usr_uart_write_string(dev->cli_device->uart_device,
					    (uint8_t*)"\tADXRS290 PMDZ application.\n");
		if(ret != SUCCESS)
			return ret;
		usr_uart_write_string(dev->cli_device->uart_device,
				      (uint8_t*)
				      "For commands with options as arguments typing the command and 'space' without arguments\n");
		if(ret != SUCCESS)
			return ret;
		usr_uart_write_string(dev->cli_device->uart_device,
				      (uint8_t*)"will show the list of options.\n");
		if(ret != SUCCESS)
			return ret;
		return usr_uart_write_string(dev->cli_device->uart_device,
					     (uint8_t*)"Available verbose commands.\n\n");
	} else {
		return usr_uart_write_string(dev->cli_device->uart_device,
					     (uint8_t*)"\nAvailable short commands:\n\n");
	}
}

/**
 * Display Gyro specific functions.
 *
 * adxrs290_pmdz_help() helper function.
 *
 * @param [in] dev	 	 	 - The device structure.
 * @param [in] short_command - True to display the long command prompt,
 *                             false to display the short command prompt.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
static int32_t adxrs290_pmdz_help_gyro(struct adxrs290_pmdz_dev *dev,
				     bool short_command)
{
	int32_t ret;

	if (!short_command) {
		ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)
					" set_lpf <loc>                                   - Set the low-pass filter pole.\n");
	}
	else
	{
		ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)
					" sl <loc>                                        - Set the low-pass filter pole.\n");
	}
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)
					"                             <loc> = location for the pole; values are:\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 0 -> 480 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 1 -> 320 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 2 -> 160 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 3 -> 80 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 4 -> 56.6 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 5 -> 40 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 6 -> 28.3 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 7 -> 20 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	if (!short_command) {
		ret = usr_uart_write_string(dev->cli_device->uart_device,
					    (uint8_t*)"                             Example: set_lpf 1\n");
		if(ret != SUCCESS)
			return ret;
		ret = usr_uart_write_string(dev->cli_device->uart_device,
					    (uint8_t*)
					    " set_hpf <loc> - Set the low-pass filter pole.\n");
		if(ret != SUCCESS)
			return ret;
	}
	else {
		ret = usr_uart_write_string(dev->cli_device->uart_device,
					    (uint8_t*)"                             Example: sl 1\n");
		if(ret != SUCCESS)
			return ret;
		ret = usr_uart_write_string(dev->cli_device->uart_device,
					    (uint8_t*)
					    " sh <loc> - Set the low-pass filter pole.\n");
		if(ret != SUCCESS)
			return ret;
	}
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)
					"                             <loc> = location for the pole; values are:\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 0 -> All pass;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 1 -> 0.011 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 2 -> 0.022 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 3 -> 0.044 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 4 -> 0.087 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 5 -> 0.175 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 6 -> 0.350 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 7 -> 0.700 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 8 -> 1.400 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 9 -> 2.800 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	ret = usr_uart_write_string(dev->cli_device->uart_device,
					(uint8_t*)"                                       - 10 -> 11.30 Hz;\n");
	if(ret != SUCCESS)
		return ret;
	if (!short_command) {
		ret = usr_uart_write_string(dev->cli_device->uart_device,
					    (uint8_t*)"                             Example: set_hpf 1\n");
		if(ret != SUCCESS)
			return ret;
		ret = usr_uart_write_string(dev->cli_device->uart_device,
					    (uint8_t*)
					    " read_rate <axis>          - Display the angular rate of axis X or Y.\n");
		if(ret != SUCCESS)
			return ret;
		ret = usr_uart_write_string(dev->cli_device->uart_device,
						(uint8_t*)
						"                           - if no axis is passed, both X and Y will be displayed.\n");
		if(ret != SUCCESS)
			return ret;
		if(ret != SUCCESS)
			return ret;
		ret = usr_uart_write_string(dev->cli_device->uart_device,
					     (uint8_t*)"                             Example: read_rate X\n");
		if(ret != SUCCESS)
			return ret;
		ret = usr_uart_write_string(dev->cli_device->uart_device,
					    (uint8_t*)
					    " prod_test                 - Run production test.\n");
		if(ret != SUCCESS)
			return ret;
		return usr_uart_write_string(dev->cli_device->uart_device,
					     (uint8_t*)"                             Example: prod_test\n");
	}
	else{
		ret = usr_uart_write_string(dev->cli_device->uart_device,
						(uint8_t*)"                             Example: sh 1\n");
		if(ret != SUCCESS)
			return ret;
		ret = usr_uart_write_string(dev->cli_device->uart_device,
						(uint8_t*)
						" r <axis>                  - Display the angular rate of axis X or Y.\n");
		if(ret != SUCCESS)
			return ret;
		ret = usr_uart_write_string(dev->cli_device->uart_device,
						(uint8_t*)
						"                           - if no axis is passed, both X and Y will be displayed.\n");
		if(ret != SUCCESS)
			return ret;
		ret = usr_uart_write_string(dev->cli_device->uart_device,
						 (uint8_t*)"                             Example: r X\n");
		if(ret != SUCCESS)
			return ret;
		ret = usr_uart_write_string(dev->cli_device->uart_device,
						(uint8_t*)
						" t                 - Run production test.\n");
		if(ret != SUCCESS)
			return ret;
		return usr_uart_write_string(dev->cli_device->uart_device,
						 (uint8_t*)"                             Example: t\n");
	}

}

/**
 * Display help options in the CLI.
 *
 * @param [in] dev - The device structure.
 * @param [in] arg - Not used in this case. It exists to keep the function
 *                   prototype compatible with the other functions that can be
 *                   called from the CLI.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t adxrs290_pmdz_help(struct adxrs290_pmdz_dev *dev, uint8_t *arg)
{
	int32_t ret;

	ret = adxrs290_pmdz_help_prompt(dev, HELP_LONG_COMMAND);
	if(ret != SUCCESS)
		return ret;

	ret = adxrs290_pmdz_help_gyro(dev, HELP_LONG_COMMAND);
	if(ret != SUCCESS)
		return ret;

	ret = adxrs290_pmdz_help_prompt(dev, HELP_SHORT_COMMAND);
	if(ret != SUCCESS)
		return ret;

	return adxrs290_pmdz_help_gyro(dev, HELP_SHORT_COMMAND);
}

/**
 * Display "Band pass filter location input (LPF/HPF)" command error.
 *
 * adxrs290_pmdz_set_lpf() helper function that displays a tooltip to using
 * this specific command.
 *
 * @param [in] dev      - Application software handler.
 *
 * @return void
 */
static void adxrs290_pmdz_arg_err(struct adxrs290_pmdz_dev *dev)
{
	usr_uart_write_string(dev->cli_device->uart_device,
			      (uint8_t*)"Invalid filter pole location. run help command for details.\n");
}

/**
 * Set low-pass filter pole location.
 *
 * @param [in] dev - Application software handler.
 * @param [in] arg - Pointer to the argument.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t adxrs290_pmdz_set_lpf(struct adxrs290_pmdz_dev *dev, uint8_t *arg)
{
	uint8_t loc;

	if(!arg) {
		adxrs290_pmdz_arg_err(dev);
		return SUCCESS;
	}

	loc = atoi((char *)arg);
	return adxrs290_set_lpf(dev->adxrs290_device, loc);
}

/**
 * Set high-pass filter pole location.
 *
 * @param [in] dev - Application software handler.
 * @param [in] arg - Pointer to the argument.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t adxrs290_pmdz_set_hpf(struct adxrs290_pmdz_dev *dev, uint8_t *arg)
{
	uint8_t loc;

	if(!arg) {
		adxrs290_pmdz_arg_err(dev);
		return SUCCESS;
	}

	loc = atoi((char *)arg);
	return adxrs290_set_hpf(dev->adxrs290_device, loc);
}

/**
 * Set high-pass filter pole location.
 *
 * @param [in] dev - Application software handler.
 * @param [in] arg - Pointer to the argument.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t adxrs290_pmdz_read_data(struct adxrs290_pmdz_dev *dev, uint8_t *arg)
{
	int32_t ret=SUCCESS;
	uint8_t axis;
	int16_t rateX, rateY;
	char buff[20];
	if(!arg) {
		axis = 0;
	}
	else
	{
		axis = arg[0];
	}
	if (axis=='x' || axis!='X') {
		ret=adxrs290_get_rate_data(dev->adxrs290_device, ADXRS290_CHANNEL_X, &rateX);
		usr_uart_write_string(dev->cli_device->uart_device,
				      (uint8_t*)"X-axis rate (LSB): ");
		sprintf(buff, "%d\n", rateX);
		usr_uart_write_string(dev->cli_device->uart_device,(uint8_t *)buff);
	}
	else if (axis=='y' || axis!='Y'){
		ret=adxrs290_get_rate_data(dev->adxrs290_device, ADXRS290_CHANNEL_Y, &rateY);
		usr_uart_write_string(dev->cli_device->uart_device,
					  (uint8_t*)"Y-axis rate (LSB): ");
		sprintf(buff, "%d\n", rateY);
				usr_uart_write_string(dev->cli_device->uart_device,(uint8_t *)buff);
	}
	else {
		ret=adxrs290_get_rate_dataXY(dev->adxrs290_device,&rateX, &rateY);
		usr_uart_write_string(dev->cli_device->uart_device,
				      (uint8_t*)"X-axis rate (LSB): ");
		sprintf(buff, "%d\n", rateX);
		usr_uart_write_string(dev->cli_device->uart_device,(uint8_t *)buff);
		usr_uart_write_string(dev->cli_device->uart_device,
					  (uint8_t*)"Y-axis rate (LSB): ");
		sprintf(buff, "%d\n", rateY);
				usr_uart_write_string(dev->cli_device->uart_device,(uint8_t *)buff);
	}
	return ret;
}

/**
 * Production test routine.
 *
 * Run the production test routine for the adxrs290.
 *
 * @param [in] dev - The device structure.
 * @param [in] arg - Not used in this case. It exists to keep the function
 *                   prototype compatible with the other functions that can be
 *                   called from the CLI.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t adxrs290_pmdz_prod_test(struct adxrs290_pmdz_dev *dev, uint8_t *arg)
{


	return SUCCESS;
}

/**
 * Provide default configuration for the application initialization handler.
 *
 * @param [out] init_param - Pointer to the initialization structure.
 *
 * @return void.
 */
void adxrs290_pmdz_get_config(struct adxrs290_pmdz_init_param *init_param)
{
	//init_param->adxrs290_init.sw_ldac.sink0 = false;
	init_param->adxrs290_init.spi_init.chip_select = 0xFF;
	init_param->adxrs290_init.spi_init.extra = NULL;
	init_param->adxrs290_init.spi_init.id = SPI_PMOD;
	init_param->adxrs290_init.spi_init.max_speed_hz = 9000000;
	init_param->adxrs290_init.spi_init.mode = SPI_MODE_0;
	init_param->adxrs290_init.spi_init.type = ADICUP3029_SPI;
	init_param->adxrs290_init.mode = ADXRS290_MODE_MEASUREMENT;
	init_param->adxrs290_init.lpf = ADXRS290_LPF_480HZ;
	init_param->adxrs290_init.hpf = ADXRS290_HPF_ALL_PASS;
	init_param->cli_init.uart_init.baudrate = bd115200;
	init_param->cli_init.uart_init.bits_no = 8;
	init_param->cli_init.uart_init.has_callback = true;
}

/**
 * Application main process.
 *
 * Runs the Command Line Interpretor.
 *
 * @param [in] dev - Pointer to the application handler.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t adxrs290_pmdz_process(struct adxrs290_pmdz_dev *dev)
{
	return cli_process(dev->cli_device);
}

/**
 * Allocate memory for the application handlers and initialize the system.
 *
 * @param [out] device     - Pointer to the application handler.
 * @param [in]  init_param - Pointer to the initialization structure.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t adxrs290_pmdz_setup(struct adxrs290_pmdz_dev **device,
			   struct adxrs290_pmdz_init_param *init_param)
{
	int32_t ret;
	struct adxrs290_pmdz_dev *dev;
	uint8_t app_name[] = {
		'A', 'D', 'X', 'R', 'S', '2', '9', '0', '_', 'P', 'M', 'D', 'Z', 0
	};

	dev = calloc(1, sizeof *dev);
	if(!dev)
		return FAILURE;

	ret = timer_start();
	if(ret != SUCCESS)
		goto error;

	ret = cli_setup(&dev->cli_device, &init_param->cli_init);
	if(ret != SUCCESS)
		goto error;
	cli_load_command_vector(dev->cli_device, adxrs290_fnc_ptr);
	cli_load_command_calls(dev->cli_device, (uint8_t **)adxrs290_fnc_calls);
	cli_load_command_sizes(dev->cli_device, adxrs290_fnc_call_size);
	cli_load_descriptor_pointer(dev->cli_device, dev);
	ret = cli_cmd_prompt(dev->cli_device, app_name);
	if(ret != SUCCESS)
		goto error;

	ret = adxrs290_init(&dev->adxrs290_device, &init_param->adxrs290_init);

	if(ret != SUCCESS)
		goto error;

	*device = dev;

	return ret;
error:
	free(dev);

	return ret;
}

/**
 * Free memory allocated by adxrs290_pmdz_setup().
 *
 * @param [in] device - Pointer to the application handler.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t adxrs290_pmdz_remove(struct adxrs290_pmdz_dev *dev)
{
	int32_t ret;

	if(!dev)
		return FAILURE;

	ret = adxrs290_remove(dev->adxrs290_device);
	if(ret != SUCCESS)
		return ret;

	ret = cli_remove(dev->cli_device);
	if(ret != SUCCESS)
		return ret;

	free(dev);

	return ret;
}
