/*
 * lps25hb.c
 *
 *  Created on: Nov 17, 2017
 *      Author: kerhoas
 */

#include "lps25hb.h"
#include "drv_i2c.h"

//========================================================================
//			PRESSURE SENSOR
//========================================================================

#define LPS25HB_I2C_ADDRESS		0xBA

//=======================================================
//			REGISTER ADDRESS MAP
//=======================================================

#define	REF_P_XL		0x08
#define	REF_P_L			0x09
#define	REF_P_H			0x0A
#define	WHO_AM_I		0x0F
#define	RES_CONF		0x10
#define	CTRL_REG1		0x20
#define	CTRL_REG2		0x21
#define	CTRL_REG3		0x22
#define	CTRL_REG4		0x23
#define	INTERRUPT_CFG	0x24
#define	INT_SOURCE		0x25
#define	STATUS_REG		0x27
#define	PRESS_OUT_XL	0x28
#define	PRESS_OUT_L		0x29
#define	PRESS_OUT_H		0x2A
#define	TEMP_OUT_L		0x2B
#define	TEMP_OUT_H		0x2C
#define	FIFO_CTRL		0x2E
#define	FIFO_STATUS		0x2F
#define	THS_P_L			0x30
#define	THS_P_H			0x31
#define	RPDS_L			0x39
#define	RPDS_H			0x3A

//========================================================================
//			WHO AM I
//========================================================================
// Default : 0xBD
uint8_t lps25hb_whoAmI()
{
	uint8_t id;
	i2c1_ReadRegBuffer(LPS25HB_I2C_ADDRESS, WHO_AM_I,  &id, 1);
	return id;
}
//========================================================================
//			SETUP
//========================================================================
void lps25hb_setup()
{
	uint8_t data;
	data = 0x90;	// Active mode, continuous update
	i2c1_WriteRegBuffer(LPS25HB_I2C_ADDRESS, CTRL_REG1,  &data, 1);

	data=0xA;	// resolution: temp=32, pressure=128
	i2c1_WriteRegBuffer(LPS25HB_I2C_ADDRESS, RES_CONF,  &data, 1);
}
//========================================================================
//			GET PRESSURE
//========================================================================
int lps25hb_getPressure(float* pressure)
{
	uint8_t data[3];
	static float press = 0.0;

	i2c1_ReadRegBuffer(LPS25HB_I2C_ADDRESS, (PRESS_OUT_XL | 0x80),  data, 3);
	press = ((data[2] * 65536) + ((data[1] * 256) + data[0])) / 4096.0;

	*pressure=press;

	return 0;
}
//========================================================================
//			GET TEMPERATURE
//========================================================================
int lps25hb_getTemperature(float* temp)
{
	uint8_t tempH, tempL;
	static float temperature = 0.0;

	//uint8_t  data = 0x1;	// For one-shot mode
	//i2c1_WriteRegBuffer(LPS25HB_I2C_ADDRESS, CTRL_REG2,  &data, 1);
	i2c1_ReadRegBuffer(LPS25HB_I2C_ADDRESS, TEMP_OUT_L,  &tempL, 1);
	i2c1_ReadRegBuffer(LPS25HB_I2C_ADDRESS, TEMP_OUT_H,  &tempH, 1);

	int16_t val = ((int16_t)tempH) << 8 | (tempL & 0xff);
	temperature = 42.5+(float)val/480.0;

	*temp = temperature;

	return 0;
}
//========================================================================








