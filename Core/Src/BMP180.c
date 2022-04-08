/*
 * BMP180.c
 *
 *  Created on: Apr 4, 2022
 *      Author: carol
 */

#include "BMP180.h"
#include "math.h"

//uint16_t AC5, AC6;
//int16_t MC, MD;

unsigned short AC5 = 0;
unsigned short AC6 = 0;

short MC = 0;
short MD = 0;

float UT = 0;
short oss = 0;
long UP = 0;
long X1 = 0;
long X2 = 0;
long X3 = 0;
long B3 = 0;
long B5 = 0;
unsigned long B4 = 0;
long B6 = 0;
unsigned long B7 = 0;

float Temp = 0;

#define BMP180_ADDRESS 0xEE
// Defines according to the datsheet

I2C_HandleTypeDef *i2cMAdd;

HAL_StatusTypeDef WriteI2C(uint8_t dev_address, uint8_t reg, uint8_t bytes, uint8_t* dado)
{
	return HAL_I2C_Mem_Write(i2cMAdd, dev_address, reg, 1, dado, bytes, 1000);
}

HAL_StatusTypeDef ReadI2C(uint8_t dev_address, uint8_t reg, uint8_t n_bytes, uint8_t* dado)
{
	return HAL_I2C_Mem_Read(i2cMAdd, dev_address, reg, 1, dado, n_bytes, 1000);
}

HAL_StatusTypeDef InitSensorTemperatureReading(I2C_HandleTypeDef *i2cAdd)
{
	i2cMAdd = i2cAdd;

	uint8_t device_id;
	ReadI2C(BMP_ADDRESS, ID_REG, 1, &device_id);

	// Calibration data read from the eprom


	uint8_t temp[2];
	ReadI2C(BMP_ADDRESS, AC5_H, 2, temp);
	AC5 = temp[0];
	AC5 = AC5 << 8 | temp[1];

	ReadI2C(BMP_ADDRESS, AC6_H, 2, temp);
	AC6 = temp[0];
	AC6 = AC6 << 8 | temp[1];

	ReadI2C(BMP_ADDRESS, MC_H, 2, temp);
	MC = temp[0];
	MC = MC << 8 | temp[1];

	ReadI2C(BMP_ADDRESS, MD_H, 2, temp);
	MD = temp[0];
	MD = MD << 8 | temp[1];

	if(device_id == 0x55){
		return HAL_OK;
	}
}

uint16_t ReadUncTemperature(void)
{
	uint8_t datatowrite = 0x2E; //Para ler sensor
	uint8_t Temp_Buff[2] = {0}; //Vetor auxiliar
	uint8_t Temp_Buff2[2] = {0}; //Vetor auxiliar

	//Escreve via I2C, no BMP, o valor para leitura 0x2E, no registrador 0xF4
	HAL_I2C_Mem_Write(i2cMAdd, BMP180_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
	HAL_Delay (5);  // Espera 4.5 ms da leitura

	HAL_I2C_Mem_Read(i2cMAdd, BMP180_ADDRESS, 0xF6, 1, Temp_Buff, 2, 1000);
	//HAL_I2C_Mem_Read(i2cMAdd, BMP180_ADDRESS, 0xF6, 1, Temp_Buff, 2, 1000);
	//HAL_I2C_Mem_Read(i2cMAdd, BMP180_ADDRESS, 0xF7, 1, Temp_Buff2, 2, 1000);
	return ((Temp_Buff[0]<<8) + Temp_Buff[1]);
}

float ReadTemperature (void)
{
	UT = ReadUncTemperature();
	X1 = ((UT-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	Temp = (B5+8)/(pow(2,4));
	Temp = Temp/10;
	return Temp;
}


void read_calliberation_data (void)
{

}



