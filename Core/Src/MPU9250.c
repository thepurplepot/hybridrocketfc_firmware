/*
 * MPU9250.c
 *
 *  Created on: Nov 18, 2021
 *      Author: William FLetcher
 *
 *      ToDo:
 *      -Mag. Calibration https://github.com/mindThomas/STM32-libraries/blob/master/Drivers/MPU9250/MPU9250.cpp    904
 *      https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
 *      DMP Programming + Setup https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/blob/master/src/util/inv_mpu.c    2776
 *      https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/blob/master/src/util/inv_mpu_dmp_motion_driver.c
 *
 */

#include "MPU9250.h"

/*
 * Low-Level Functions
 */
HAL_StatusTypeDef MPU9250_ReadRegister (MPU9250 *dev, uint8_t reg, uint8_t *data) {
	return HAL_I2C_Mem_Read( dev-> i2cHandle, MPU9250_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU9250_ReadRegisters (MPU9250 *dev, uint8_t reg, uint8_t *data, uint8_t length) {
	return HAL_I2C_Mem_Read( dev-> i2cHandle, MPU9250_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU9250_WriteRegister (MPU9250 *dev, uint8_t reg, uint8_t *data) {
	return HAL_I2C_Mem_Write( dev-> i2cHandle, MPU9250_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef AK8963_ReadRegister (MPU9250 *dev, uint8_t reg, uint8_t *data) {
	return HAL_I2C_Mem_Read( dev-> i2cHandle, AK8963_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef AK8963_ReadRegisters (MPU9250 *dev, uint8_t reg, uint8_t *data, uint8_t length) {
	return HAL_I2C_Mem_Read( dev-> i2cHandle, AK8963_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef AK8963_WriteRegister (MPU9250 *dev, uint8_t reg, uint8_t *data) {
	return HAL_I2C_Mem_Write( dev-> i2cHandle, AK8963_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

uint8_t MPU9250_Init (MPU9250 *dev, I2C_HandleTypeDef *i2cHandle) {
	dev->i2cHandle = i2cHandle;

	dev->acc_mps2[0] = 0.0f;
	dev->acc_mps2[1] = 0.0f;
	dev->acc_mps2[2] = 0.0f;

	dev->temp_C = 0.0f;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	uint8_t regData;
	status = MPU9250_ReadRegister( dev, MPU9250_WHO_AM_I, &regData);
	errNum += (status != HAL_OK);
	if (regData != MPU9250_DEVICE_ID) {
		return 255; // Device has unexpected ID
	}

	/* Reset chip */
	regData = 0x80; // SLEEP = 1 (p. 40)
	status = MPU9250_WriteRegister (dev, MPU9250_PWR_MGMT_1, &regData);
	errNum += (status != HAL_OK);
	HAL_Delay(100);
	/* Wake up chip. */
	regData = 0x01; // SLEEP = 0 (p. 40), CLOCK reference is gyro
	status = MPU9250_WriteRegister (dev, MPU9250_PWR_MGMT_1, &regData);
	errNum += (status != HAL_OK);
	/* Config. Gyro. */
	regData = 0x04; // DLPF_CFG = 4 20Hz(p. 13)
	status = MPU9250_WriteRegister (dev, MPU9250_CONFIG, &regData);
	errNum += (status != HAL_OK);
	regData = 0x08; // GYRO_FS_SEL = 01 +-500dps (p. 14)
	status = MPU9250_WriteRegister (dev, MPU9250_GYRO_CONFIG, &regData);
	errNum += (status != HAL_OK);
	/* Config. Accel. */
	regData = 0x08; // ACCEL_FS_SEL = 01 +-4g (p. 14)
	status = MPU9250_WriteRegister (dev, MPU9250_ACCEL_CONFIG, &regData);
	errNum += (status != HAL_OK);
	regData = 0x04; // A_DLPFCFG = 4 21Hz(p. 15)
	status = MPU9250_WriteRegister (dev, MPU9250_ACCEL_CONFIG_2, &regData);
	errNum += (status != HAL_OK);
	/* Config. Mag. */
	regData = 0x02; // BYPASS_EN
	status = MPU9250_WriteRegister (dev, MPU9250_INT_PIN_CFG, &regData);
	errNum += (status != HAL_OK);
	regData = 0x1F; // Fuse ROM access mode
	status = AK8963_WriteRegister (dev, AK8963_CNTL, &regData);
	errNum += (status != HAL_OK);
	HAL_Delay(100); // Wait for mode change
	//READ FROM MAG FOR SENSITIVITY ADJ.
	uint8_t asa[3];
	status = AK8963_ReadRegisters (dev, AK8963_ASAX, asa, 3);
	errNum += (status != HAL_OK);
	dev->mag_asa[0] = (asa[0] - 128)*0.5/128+1;
	dev->mag_asa[1] = (asa[1] - 128)*0.5/128+1;
	dev->mag_asa[2] = (asa[2] - 128)*0.5/128+1;
	regData = 0x00; // Power down mode
	status = AK8963_WriteRegister (dev, AK8963_CNTL, &regData); //CNTL1_AD
	errNum += (status != HAL_OK);
	HAL_Delay(100); // Wait for mode change
	regData = 0x16; // Continuous mode
	status = AK8963_WriteRegister (dev, AK8963_CNTL, &regData); //CNTL1_AD
	errNum += (status != HAL_OK);
	HAL_Delay(100); // Wait for mode change
	/* Enable the Int pin */
	regData = 0x01; // RAW_RDY_EN = 1 (p. 30)
	status = MPU9250_WriteRegister (dev, MPU9250_INT_ENABLE, &regData);
	errNum += (status != HAL_OK);

	return errNum;
}

HAL_StatusTypeDef MPU9250_ClearInt ( MPU9250 *dev ) {
	uint8_t regData;
	regData = 0x00;
	HAL_StatusTypeDef status = MPU9250_WriteRegister (dev, MPU9250_INT_STATUS, &regData);
	return status;
}

HAL_StatusTypeDef MPU9250_ReadTemp ( MPU9250 *dev ) {
	uint8_t regData[2];
	HAL_StatusTypeDef status = MPU9250_ReadRegisters(dev, MPU9250_TEMP_OUT_H, regData, 2);
	int16_t tempRaw = (((regData[0] & 0x0F) << 4) | regData[1]);
	dev->temp_C = ((float)tempRaw - 0.0f)/321.0f + 21.0f;
	return status;
}

HAL_StatusTypeDef MPU9250_ReadAccel ( MPU9250 *dev ) {
	uint8_t regData[6];
	HAL_StatusTypeDef status = MPU9250_ReadRegisters(dev, MPU9250_ACCEL_XOUT_H, regData, 6);
	int16_t accelRawSigned[3];
	accelRawSigned[0] = (regData[0] << 8) | regData[1];
	accelRawSigned[1] = (regData[2] << 8) | regData[3];
	accelRawSigned[2] = (regData[4] << 8) | regData[5];
	// Range +- 4G
	dev->acc_mps2[0] = 9.81f * accelRawSigned[0] / 8192.0f;
	dev->acc_mps2[1] = 9.81f * accelRawSigned[1] / 8192.0f;
	dev->acc_mps2[2] = 9.81f * accelRawSigned[2] / 8192.0f;
	return status;
}

HAL_StatusTypeDef MPU9250_ReadGyro ( MPU9250 *dev ) {
	uint8_t regData[6];
	HAL_StatusTypeDef status = MPU9250_ReadRegisters(dev, MPU9250_GYRO_XOUT_H, regData, 6);
	int16_t gyroRawSigned[3];
	gyroRawSigned[0] = (regData[0] << 8) | regData[1];
	gyroRawSigned[1] = (regData[2] << 8) | regData[3];
	gyroRawSigned[2] = (regData[4] << 8) | regData[5];
	// Range +- 500dps
	dev->gyro_dps[0] = gyroRawSigned[0] / 32.8f;
	dev->gyro_dps[1] = gyroRawSigned[1] / 32.8f;
	dev->gyro_dps[2] = gyroRawSigned[2] / 32.8f;
	return status;
}

HAL_StatusTypeDef MPU9250_ReadMag ( MPU9250 *dev ) {
	uint8_t regData[7];
	HAL_StatusTypeDef status = AK8963_ReadRegister (dev, AK8963_ST1, regData);
	if( regData[0] & 0x01 ) {
		status = AK8963_ReadRegisters (dev, AK8963_HXL, regData, 7);
		if( !(regData[6] & 0x08) ) { //Check AK8963_ST2
			int16_t magRawSigned[3];
			magRawSigned[0] = regData[0]  | (regData[1] << 8);
			magRawSigned[1] = regData[2]  | (regData[3] << 8);
			magRawSigned[2] = regData[4]  | (regData[5] << 8);
			// Range +-
			dev->mag_mt[0] = 0.1499f * dev->mag_asa[0] * magRawSigned[0]; // 4912/32760 uT/tick
			dev->mag_mt[1] = 0.1499f * dev->mag_asa[1] * magRawSigned[1];
			dev->mag_mt[2] = 0.1499f * dev->mag_asa[2] * magRawSigned[2];
			//Include calibration values somehow (subtract bias, multiply by scale);
		}
	}
	return status;
}

HAL_StatusTypeDef MPU9250_CalMag (MPU9250 *dev) {
	// https://github.com/mindThomas/STM32-libraries/blob/master/Drivers/MPU9250/MPU9250.cpp
	//FLT_MIN         = 1.175494e-38
	//FLT_MAX         = 3.402823e+38
	float mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	float mag_max[3] = { FLT_MIN, FLT_MIN, FLT_MIN }, mag_min[3] = { FLT_MAX, FLT_MAX, FLT_MAX };
	HAL_StatusTypeDef status;

	// Wave device in figure of eight during this process

	//Determin max and min outputs during wave
	uint16_t sample_count = 1500; //15 seconds of data at 100Hz
	for(uint16_t ii = 0; ii < sample_count; ii++){
		status = MPU9250_ReadMag ( MPU9250 *dev );
		for(uint16_t jj = 0; jj < 3; j++){
			if (dev->mag_mt[jj] > mag_max[jj]) mag_max[jj] = dev->mag_mt[jj];
			if (dev->mag_mt[jj] < mag_min[jj]) mag_min[jj] = dev->mag_mt[jj];
		}

		HAL_Delay(12);
	}

	// Hard Iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;

	// Soft iron correction
	mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;
	mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;
	mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;

	float avg_scale = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3;
	mag_scale[0] = avg_scale / mag_scale[0];
	mag_scale[1] = avg_scale / mag_scale[1];
	mag_scale[2] = avg_scale / mag_scale[2];

	//dev->mag_bias[0] = mag_bias[0]; something like this

	return status;
	//Output bias and scale arrays to be used when reading mag data.
}


// DMP
uint8_t MPU9250_InitDMP( MPU9250 *dev ) {
	// https://github.com/kriswiner/MPU9250/blob/master/Documents/Application%20Note%20-%20Programming%20Sequence%20for%20DMP%20Hardware%20Functions%20v12%20(....pdf
	/* Reset chip */
	regData = 0x80; // SLEEP = 1 (p. 40)
	status = MPU9250_WriteRegister (dev, MPU9250_PWR_MGMT_1, &regData);
	errNum += (status != HAL_OK);
	HAL_Delay(100);
	/* Wake up chip / Configure Power Management */
	regData = 0x6B;
	status = MPU9250_WriteRegister (dev, MPU9250_PWR_MGMT_1, &regData);
	errNum += (status != HAL_OK);
	regData = 0x6C;
	status = MPU9250_WriteRegister (dev, MPU9250_PWR_MGMT_2, &regData);
	errNum += (status != HAL_OK);
	/* Configure Gyro. */
	regData = 0x03; // 42Hz LPF
	status = MPU9250_WriteRegister (dev, MPU9250_CONFIG, &regData);
	errNum += (status != HAL_OK);
	regData = 0x18; // +-2000dps FSR
	status = MPU9250_WriteRegister (dev, MPU9250_GYRO_CONFIG, &regData);
	errNum += (status != HAL_OK);
	/* Configure Accel. */
	regData = 0x08; // +-4g FSR
	status = MPU9250_WriteRegister (dev, MPU9250_ACCEL_CONFIG, &regData);
	errNum += (status != HAL_OK);
	/* Configure FIFO / Interupts */
	regData = 0x00; // Defers FIFO control to DMP
	status = MPU9250_WriteRegister (dev, MPU9250_FIFO_EN, &regData);
	errNum += (status != HAL_OK);
	regData = 0x00; // Defers interupt control to DMP
	status = MPU9250_WriteRegister (dev, MPU9250_INT_ENABLE, &regData);
	errNum += (status != HAL_OK);
	/* Reset FIFO */
	regData = 0x04;
	status = MPU9250_WriteRegister (dev, MPU9250_USER_CTRL, &regData);
	errNum += (status != HAL_OK);
	/* Configure Sample Rate */
	regData = 0x04; // 200Hz
	status = MPU9250_WriteRegister (dev, MPU9250_SMPLRT_DIV, &regData);
	errNum += (status != HAL_OK);
	/* Load DMP Firmware */
	// AA3-AA6 {0x20 0x28, 0x30, 0x30}
	regData = 0x0A; // Write starting at AA3
	status = MPU9250_WriteRegister (dev, DMP_CTRL_1, &regData);
	errNum += (status != HAL_OK);
	regData = 0xA3; // Write starting at AA3
	status = MPU9250_WriteRegister (dev, DMP_CTRL_2, &regData);
	errNum += (status != HAL_OK);
	regData = 0x20; // Enable 6-axis quaternion
	status = MPU9250_WriteRegister (dev, DMP_CTRL_3, &regData);
	errNum += (status != HAL_OK);
	//FIRMWARE START VALUE

	/* Start DMP */
	regData = 0x40;
	status = MPU9250_WriteRegister (dev, MPU9250_USER_CTRL, &regData);
	errNum += (status != HAL_OK);
	regData = 0x04;
	status = MPU9250_WriteRegister (dev, MPU9250_USER_CTRL, &regData);
	errNum += (status != HAL_OK);
	regData = 0x80;
	status = MPU9250_WriteRegister (dev, MPU9250_USER_CTRL, &regData);
	errNum += (status != HAL_OK);
	regData = 0x08;
	status = MPU9250_WriteRegister (dev, MPU9250_USER_CTRL, &regData);
	errNum += (status != HAL_OK);
	regData = 0x02;
	status = MPU9250_WriteRegister (dev, MPU9250_INT_ENABLE, &regData);
	errNum += (status != HAL_OK);



	return errNum;

}
