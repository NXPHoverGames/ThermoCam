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
 * @file hg_amg88xx.cpp
 * AMG8833 temperature sensor for I2C
 *
 * @author Katrin Moritz
 * @author Leo Mustafa
 */

#include "hg_amg88xx.h"
#include <px4_platform_common / getopt.h>
#include <stdio.h>
using namespace std;

extern "C" __EXPORT int hg_amg88xx_main(int argc, char *argv[]);

/* Constructor */
HG_AMG88xx::HG_AMG88xx() : I2C("AMG88xx", "/dev/amg88xx", PX4_I2C_BUS_EXPANSION, AMG88xx_I2CADDR, AMG88xx_BUS_SPEED)
{

	HG_AMG88xx::init();

	//enter normal mode
	_pctl.PCTL = AMG88xx_NORMAL_MODE;
	uint8_t mode[2] = {(uint8_t)(AMG88xx_PCTL), _pctl.get()};
	transfer(mode, sizeof(mode), nullptr, 0);

	//software reset
	_rst.RST = AMG88xx_INITIAL_RESET;
	uint8_t reset[2] = {(uint8_t)(AMG88xx_RST), _rst.get()};
	transfer(reset, sizeof(reset), nullptr, 0);

	//disable interrupts by default
	HG_AMG88xx::disableInterrupt();

	//set to 10 FPS
	_fpsc.FPS = AMG88xx_FPS_10;
	uint8_t fps[2] = {(uint8_t)(AMG88xx_FPSC), _fpsc.get()};
	transfer(fps, sizeof(fps), nullptr, 0);

	usleep(100);
}

/* Initialization of I2C device */
int HG_AMG88xx::init()
{
	int ret = OK;

	/* do I2C init (and probe) first */
	ret = I2C::init();

	/* if probe/setup failed, bail now */
	if (ret != OK)
	{
		DEVICE_DEBUG("I2C setup failed");
		return ret;
	};

	return ret;
}

/* Enable Interrupt */
void HG_AMG88xx::enableInterrupt()
{
	_intc.INTEN = 1;
	uint8_t cmd[2] = {(uint8_t)(AMG88xx_INTC), _intc.get()};
	transfer(cmd, sizeof(cmd), nullptr, 0);
};

/* Disable Interrupt */
void HG_AMG88xx::disableInterrupt()
{
	_intc.INTEN = 0;
	uint8_t cmd[2] = {(uint8_t)(AMG88xx_INTC), _intc.get()};
	transfer(cmd, sizeof(cmd), nullptr, 0);
};

/* Read Pixels */
void HG_AMG88xx::readPixels(float *buf, uint8_t size)
{
	uint16_t recast;
	float converted;
	// checking number of bytes to read
	uint8_t bytesToRead = ((uint8_t)(size << 1) < (uint8_t)(AMG88xx_PIXEL_ARRAY_SIZE << 1)) ? (uint8_t)(size << 1) : (uint8_t)(AMG88xx_PIXEL_ARRAY_SIZE << 1);
	uint8_t data[bytesToRead] = {0};

	const uint8_t cmd = AMG88xx_PIXEL_OFFSET;

	if (OK == transfer(&cmd, 1, data, bytesToRead))
	{

		for (int i = 0; i < size; i++)
		{
			uint8_t pos = i << 1;
			recast = ((uint16_t)data[pos + 1] << 8) | ((uint16_t)data[pos]);

			converted = int12ToFloat(recast) * (float)AMG88xx_PIXEL_TEMP_CONVERSION;
			buf[i] = converted;
		}
	}
	else
	{
		PX4_ERR("No Data received!");
	}
};

/* Read thermistor temperature */
float HG_AMG88xx::readThermistor()
{
	uint8_t raw[2];
	const uint8_t cmd = AMG88xx_TTHL;
	transfer(&cmd, 1, raw, 2);
	uint16_t recast = ((uint16_t)raw[1] << 8) | ((uint16_t)raw[0]);

	return signedMag12ToFloat(recast) * (float)AMG88xx_THERMISTOR_CONVERSION;
};

/* main */
int hg_amg88xx_main(int argc, char *argv[])
{
	PX4_INFO("Hello Hovergames TEMP2!");

	HG_AMG88xx temp_amg;

	float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
	int j = 0;

	sleep(1); // sensor boot

	do
	{
		temp_amg.readPixels(pixels, AMG88xx_PIXEL_ARRAY_SIZE);

		printf("[");

		for (int i = 1; i <= AMG88xx_PIXEL_ARRAY_SIZE; i++)
		{
			printf("%2.2f", (double)pixels[i - 1]);
			printf(", ");

			if (i % 8 == 0)
			{
				printf("\n");
			}
		}

		printf("]");
		printf("\n");
		j += 1;
		sleep(1);
	} while (j < 10);

	PX4_INFO("Hovergames TEMP2 exit");

	return 0;
} /* end: main */