/*
 * MS5637.h
 *
 *  Created on: Dec 21, 2021
 *      Author: will
 */

#ifndef INC_MS5637_H_
#define INC_MS5637_H_

#include "stm32f4xx_hal.h" // Needed for I2C

/*
 * Defines
 */
#define MS5637_I2C_ADDR (0x76 << 1) // 0x66 write, 0x77 read

#define MS5637_RESOLUTION_OSR 4096


/*
 * Commands
 */
#define MS5637_RESET 0x1E
#define MS5637_START_PRESSURE_CONVERSION 0x40
#define MS5637_START_TEMP_CONVERSION 0x50
#define MS5637_ADC_READ 0x00
#define MS5637_PROM_READ 0xA0 // 0xA0 -> 0xAE


/*
 * Sensor struct
 */

typedef struct {
	I2C_HandleTypeDef *i2cHandle;

	uint8_t coeffs[7];

	float pressure;
} MS5637;

uint8_t MS5637_Init (MS5637 *dev, I2C_HandleTypeDef *i2cHandle);

HAL_StatusTypeDef MS5637_SendCommand (MS5637 *dev, uint8_t *cmd);
HAL_StatusTypeDef MS5637_ReadPackets (MS5637 *dev, uint8_t *data, uint8_t length);

uint8_t MS5637_SetResolution (MS5637 *dev);
uint8_t MS5637_ReadTempAndPresure (MS5637 *dev);


#endif /* INC_MS5637_H_ */
