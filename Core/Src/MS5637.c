/*
 * MS5637.c
 *
 *  Created on: Dec 21, 2021
 *      Author: will
 */

uint8_t MS5637_Init (MS5637 *dev, I2C_HandleTypeDef *i2cHandle) {
	dev->i2cHandle = i2cHandle;

	dev->pressure = 0;

	HAL_StatusTypeDef status;

	uint8_t cmd;
	/* Reset chip */
	cmd = MS5637_RESET;
	status = MS5637_SendCommand(dev, &cmd);
	/* Read Claibration Data */
	status = MS5637_ReadEEPROM(dev);

	return errNum;
}


HAL_StatusTypeDef MS5637_SendCommand (MS5637 *dev, uint8_t *cmd) {
	return HAL_I2C_Master_Transmit( dev-> i2cHandle, MS5637_I2C_ADDR, cmd, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MS5637_ReadPackets (MS5637 *dev, uint8_t *data, uint8_t length) {
	return HAL_I2C_Master_Recive( dev-> i2cHandle, MS5637_I2C_ADDR, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MS5637_ReadEEPROMCoeff (MS5637 *dev, uint8_t *cmd, uint16_t *coeff) {
	HAL_StatusTypeDef status;
	uint8_t data[2];
	status = MS5637_SendCommand(dev, cmd);
	status = MS5637_ReadPackets(dev, &data, 2);
	coeff = (data[0] << 8) | data[1];
	return status;
}

HAL_StatusTypeDef MS5637_ReadEEPROM (MS5637 *dev) {
	HAL_StatusTypeDef status;
	uint8_t coeff;
	for ( uint8_t i = 0; i < 7; i++ ) {
		status = MS5637_ReadEEPROMCoeff ( dev, MS5637_PROM_READ + i*2, &coeff);
		if(status != HAL_OK)
			return status;
		dev -> coeffs + i = coeff;
	}
	// CRC Check
	return status;
}

HAL_StatusTypeDef MS5637_ConvertADC (MS5637 *dev, uint8_t *cmd, uint32_t *adc) {
	HAL_StatusTypeDef status;
	uint8_t data[3];
	status = MS5637_SendCommand(dev, cmd);
	//Delay conversion time depending on osr
	status = MS5637_SendCommand(dev, MS5637_ADC_READ);
	status = MS5637_ReadPackets(dev, &data, 3);
	*adc = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
	return status;
}


HAL_StatusTypeDef MS5637_ReadTempAndPressure (MS5637 *dev) {
	HAL_StatusTypeDef status;
	uint8_t cmd;
	uint32_t adc_temp, adc_pressure;
	// Read Temp
	cmd = MS5637_RESOLUTION_OSR*2;
	cmd |= MS5637_START_TEMP_CONVERSION;
	status = MS5637_ConvertADC(dev, *cmd, &adc_temp);
	// Read Pressure
	cmd = MS5637_RESOLUTION_OSR*2;
	cmd |= MS5637_START_PRESSURE_CONVERSION;
	status = MS5637_ConvertADC(dev, *cmd, &adc_pressure);
	// Calculate temprature

	//Second order Tempratiure compensation

	return status;
}

