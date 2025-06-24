/*
 * LSM6DS0.c
 *
 *  Created on: Nov 17, 2017
 *      Author: kerhoas
 */

#include "lsm6ds0.h"
#include "drv_i2c.h"

//=========================================================================================
//					WHO AM I
//=========================================================================================
// Default : 0x68
uint8_t lsm6ds0_whoAmI()
{
	uint8_t id;
	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_WHO_AM_I_ADDR,  &id, 1);
	return id;
}
//=========================================================================================
//					INIT
//=========================================================================================
// lsm6ds0_setup(LSM6DS0_G_ODR_119HZ, LSM6DS0_G_FS_2000,  LSM6DS0_XL_ODR_119HZ, LSM6DS0_XL_FS_2G)
void lsm6ds0_setup(uint8_t gyro_data_rate, uint8_t gyro_full_scale,  uint8_t acc_data_rate, uint8_t acc_full_scale)
{
	/******* Gyroscope init *******/
	uint8_t tmp1;
	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_CTRL_REG1_G,  &tmp1, 1);
	/* Output Data Rate selection */
    tmp1 &= ~(LSM6DS0_G_ODR_MASK);
    tmp1 |= gyro_data_rate; 	//tmp1 |= LSM6DS0_G_ODR_119HZ;
	/* Full scale selection */
	tmp1 &= ~(LSM6DS0_G_FS_MASK);
	tmp1 |= gyro_full_scale ;	//tmp1 |= LSM6DS0_G_FS_2000;
	i2c1_WriteRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_CTRL_REG1_G,  &tmp1, 1);

	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_CTRL_REG4,  &tmp1, 1);
	/* Enable X axis selection */
    tmp1 &= ~(LSM6DS0_G_XEN_MASK);
    tmp1 |= LSM6DS0_G_XEN_ENABLE;
    /* Enable Y axis selection */
    tmp1 &= ~(LSM6DS0_G_YEN_MASK);
    tmp1 |= LSM6DS0_G_YEN_ENABLE;
    /* Enable Z axis selection */
    tmp1 &= ~(LSM6DS0_G_ZEN_MASK);
    tmp1 |= LSM6DS0_G_ZEN_ENABLE;
	i2c1_WriteRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_CTRL_REG4,  &tmp1, 1);

	/***** Accelerometer init *****/

	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_CTRL_REG6_XL,  &tmp1, 1);
	/* Output Data Rate selection */
	tmp1 &= ~(LSM6DS0_XL_ODR_MASK);
	tmp1 |= acc_data_rate;		// tmp1 |= LSM6DS0_XL_ODR_119HZ;

	/* Full scale selection */
	tmp1 &= ~(LSM6DS0_XL_FS_MASK);
	tmp1 |= acc_full_scale ;  	//tmp1 |= LSM6DS0_XL_FS_2G;
	i2c1_WriteRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_CTRL_REG6_XL,  &tmp1, 1);

	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_CTRL_REG5_XL,  &tmp1, 1);
	/* Enable X axis selection */
	tmp1 &= ~(LSM6DS0_XL_XEN_MASK);
	tmp1 |= LSM6DS0_XL_XEN_ENABLE;

	/* Enable Y axis selection */
	tmp1 &= ~(LSM6DS0_XL_YEN_MASK);
	tmp1 |= LSM6DS0_XL_YEN_ENABLE;

	/* Enable Z axis selection */
	tmp1 &= ~(LSM6DS0_XL_ZEN_MASK);
	tmp1 |= LSM6DS0_XL_ZEN_ENABLE;
	i2c1_WriteRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_CTRL_REG5_XL,  &tmp1, 1);
}
//=========================================================================================
//					GYRO
//=========================================================================================
int16_t lsm6ds0_gyro_getAxesXRaw(void)
{
	uint8_t data[2];
	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_X_L_G | (1 << 7), data, 2);
	return (int16_t)(data[1]<<8 | data[0]);
}

int16_t lsm6ds0_gyro_getAxesYRaw(void)
{
	uint8_t data[2];
	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_Y_L_G | (1 << 7), data, 2);
	return (int16_t)(data[1]<<8 | data[0]);
}

int16_t lsm6ds0_gyro_getAxesZRaw(void)
{
	uint8_t data[2];
	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_Z_L_G | (1 << 7), data, 2);
	return (int16_t)(data[1]<<8 | data[0]);
}
//=========================================================================================
float lsm6ds0_getGyroScale(void){

  	float sensitivity=0;
	uint8_t scale ;
	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_CTRL_REG1_G,  &scale, 1);

	scale &= LSM6DS0_G_FS_MASK;
    switch(scale)
	{
		case LSM6DS0_G_FS_245:
			sensitivity = 8.75;
			break;
		case LSM6DS0_G_FS_500:
			sensitivity = 17.50;
			break;
		case LSM6DS0_G_FS_2000:
			sensitivity = 70;
			break;
    }
	return sensitivity;
}
//=========================================================================================
int16_t lsm6ds0_gyro_getAxesX(void)
{
	int16_t mx;
	uint8_t data[2];

	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_X_L_G | (1 << 7), data, 2);
	mx = (int16_t)(data[1]<<8 | data[0]);

	float sensitivity = lsm6ds0_getGyroScale();
	return mx * sensitivity;
}

int16_t lsm6ds0_gyro_getAxesY(void)
{
	int16_t my;
	uint8_t data[2];

	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_Y_L_G | (1 << 7), data, 2);
	my = (int16_t)(data[1]<<8 | data[0]);

	float sensitivity = lsm6ds0_getGyroScale();
	return my * sensitivity;
}

int16_t lsm6ds0_gyro_getAxesZ(void)
{
	int16_t mz;
	uint8_t data[2];

	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_Z_L_G | (1 << 7), data, 2);
	mz = (int16_t)(data[1]<<8 | data[0]);

	float sensitivity = lsm6ds0_getGyroScale();
	return mz * sensitivity;
}


//=========================================================================================
//					ACCELEROMETER
//=========================================================================================
int16_t lsm6ds0_acc_getAxesXRaw(void)
{
	uint8_t data[2];
	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_X_L_XL | (1 << 7), data, 2);
	return (int16_t)(data[1]<<8 | data[0]);
}

int16_t lsm6ds0_acc_getAxesYRaw(void)
{
	uint8_t data[2];
	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_Y_L_XL | (1 << 7), data, 2);
	return (int16_t)(data[1]<<8 | data[0]);
}

int16_t lsm6ds0_acc_getAxesZRaw(void)
{
	uint8_t data[2];
	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_Z_L_XL | (1 << 7), data, 2);
	return (int16_t)(data[1]<<8 | data[0]);
}
//=========================================================================================
float lsm6ds0_getAccScale(void){

  	float sensitivity=0;
	uint8_t acc_scale;
	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_CTRL_REG6_XL,  &acc_scale, 1);
    acc_scale &= LSM6DS0_XL_FS_MASK;

    switch(acc_scale)
	{
		case LSM6DS0_XL_FS_2G:
			sensitivity = 0.061;
			break;
		case LSM6DS0_XL_FS_4G:
			sensitivity = 0.122;
			break;
		case LSM6DS0_XL_FS_8G:
			sensitivity = 0.244;
			break;
	}
	return sensitivity;
}
//=========================================================================================

int16_t lsm6ds0_acc_getAxesX(void)
{
	int16_t mx ;
	uint8_t data[2];

	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_X_L_XL | (1 << 7), data, 2);
	mx = (int16_t)(data[1]<<8 | data[0]);

	float sensitivity = lsm6ds0_getAccScale();
	return mx * sensitivity;
}

int16_t lsm6ds0_acc_getAxesY(void)
{
	int16_t my ;
	uint8_t data[2];

	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_Y_L_XL | (1 << 7), data, 2);
	my = (int16_t)(data[1]<<8 | data[0]);

	float sensitivity = lsm6ds0_getAccScale();
	return my * sensitivity;
}

int16_t lsm6ds0_acc_getAxesZ(void)
{
	int16_t mz ;
	uint8_t data[2];

	i2c1_ReadRegBuffer(LSM6DS0_XG_MEMS_ADDRESS, LSM6DS0_XG_OUT_Z_L_XL | (1 << 7), data, 2);
	mz = (int16_t)(data[1]<<8 | data[0]);

	float sensitivity = lsm6ds0_getAccScale();
	return mz * sensitivity;
}

//=========================================================================================






