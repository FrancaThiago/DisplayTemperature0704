/*
 * BMP180.h
 *
 *  Created on: Apr 4, 2022
 *      Author: carol
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"

#define BMP_ADDRESS		0xEE
#define CONTROL_ADDRESS	0xF4
#define READ_TEMP		0xF6
#define CONFIG_TEMP		0x2E
#define ID_REG			0xD0
#define AC5_H			0xB2
#define AC5_L			0xB3
#define AC6_H			0xB4
#define AC6_L			0xB5
#define MC_H			0xBC
#define MC_L			0xBD
#define MD_H			0xBE
#define MD_L			0xBF

HAL_StatusTypeDef InitSensorTemperatureReading(I2C_HandleTypeDef *i2cAdd);

void read_calliberation_data (void);
uint16_t ReadUncTemperature(void);
float ReadTemperature (void);


#endif /* INC_BMP180_H_ */
