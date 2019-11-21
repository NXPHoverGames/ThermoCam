/****************************************************************************
 *
 *   Copyright 2019 NXP.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file hg_temp.cpp
 *
 * Example app for temperature measurement with Melaxis MLX90614 using I2C
 *
 * @author Katrin Moritz
 * @author Leo Mustafa
 */

/* Includes */
#include "hg_temp.h"
#include <px4_platform_common/getopt.h>

extern "C" __EXPORT int hg_temp_main(int argc, char *argv[]);

/* Constructor */
HG_Temp::HG_Temp() : I2C("MLX90614", "/dev/mlx90614", PX4_I2C_BUS_EXPANSION, MLX90614_I2CADDR, MLX90614_BUS_SPEED)
{
	HG_Temp::init();
}

/* Initialization of I2C device */
int HG_Temp::init()
{
	int ret = OK;

	/* do I2C init (and probe) first */
	ret = I2C::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("I2C setup failed");
		return ret;
	};

	return ret;
}

/* Read temperature from I2C bus */
double HG_Temp::readTemp(uint8_t reg)
{
	double temp = 0;
	const uint8_t cmd = reg;
	uint8_t data[4] = {0};
	uint16_t data_temp = 0;

	if (OK != transfer(&cmd, 1, data, 4)) {
		PX4_ERR("No Data received");

	} else {
		// switching read bytes
		data_temp = (data[0] | data[1] << 8);
		temp = data_temp;
		// Conversion to degrees Celsius
		temp *= .02;
		temp -= 273.15;
	}

	return temp;
} /* end: readTemp */

/* read object temperature */
double HG_Temp::readObjectTempC(void)
{
	return readTemp(MLX90614_TOBJ1);
}

/* read ambient temperature */
double HG_Temp::readAmbientTempC(void)
{
	return readTemp(MLX90614_TA);
}

/* main */
int hg_temp_main(int argc, char *argv[])
{
	PX4_INFO("Hello Hovergames TEMP!");

	HG_Temp temp;

	int counter = 20;

	// prints ambient and object temperature in console 20 times
	printf("%02i |  Ambient Temp |  Object Temp\n", counter);
	printf("-----------------------------------\n");

	for (int i = 1; i <= counter; i++) {
		printf("%02i |  %+2.2f  |  %+2.2f  \n", i, temp.readAmbientTempC(), temp.readObjectTempC());
		sleep(1);
	}

	PX4_INFO("Hovergames TEMP exit"); // print in console 

	return 0;
} /* end: main */