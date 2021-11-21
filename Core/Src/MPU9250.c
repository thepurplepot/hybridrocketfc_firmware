/*
 * MPU9250.c
 *
 *  Created on: Nov 18, 2021
 *      Author: William FLetcher
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
	dev->acc_mps2[0] = 9.81f * 0.000122070f * accelRawSigned[0];
	dev->acc_mps2[1] = 9.81f * 0.000122070f * accelRawSigned[1];
	dev->acc_mps2[2] = 9.81f * 0.000122070f * accelRawSigned[2];
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
			dev->mag_mt[0] = 0.1499f * dev->mag_asa[0] * magRawSigned[0];
			dev->mag_mt[1] = 0.1499f * dev->mag_asa[1] * magRawSigned[1];
			dev->mag_mt[2] = 0.1499f * dev->mag_asa[2] * magRawSigned[2];
		}
	}
	return status;
}
