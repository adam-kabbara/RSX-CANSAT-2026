/*
 * INA219.c
 *
 *  Created on: May 2, 2026
 *      Author: Kenneth Freeman
 */

#include "INA219.hpp"
#include "main.h"
#include <cstdint>

#define GPIOA ((GPIO_TypeDef *) GPIOA_BASE)
#define INA219_DEFAULT_ADDR 0x40

extern I2C_HandleTypeDef hi2c1;

double currentLSB = 0;

bool INA219setup(double maxCurrent, double shuntResistance, uint16_t overwriteCalibrationValue)
{
	//check if I2C is ready
	if(HAL_I2C_IsDeviceReady(&hi2c1, INA219_DEFAULT_ADDR<<1, 10, 100) != 0x00){return false;}
	//calibration
	uint16_t cal = overwriteCalibrationValue;

	if(cal == 0){
		currentLSB = maxCurrent/(16384);
		cal = 0.04096/(currentLSB*shuntResistance);
		currentLSB = 0.04096/(cal*shuntResistance); //correct LSB for truncation error
	}
	//write to calibration register

	uint8_t* caldata = (uint8_t*)&cal;
	uint8_t data_stream[2] = {caldata[1], caldata[0]};
	auto successful = HAL_I2C_Mem_Write(&hi2c1, INA219_DEFAULT_ADDR<<1, 0x05, 1, data_stream, 2, 100);
	//check for errors
	if(successful != 0x0)
	{
		return false;
	}
	return true;
}

void INA219overwriteConfiguration(uint16_t configurationValue)
{
	uint8_t* data = (uint8_t*)&configurationValue;
	uint8_t data_stream[2] = {data[1], data[0]};
	HAL_I2C_Mem_Write(&hi2c1, INA219_DEFAULT_ADDR<<1, 0x00, 1, data_stream, 2, 100);
}


uint16_t INA219getConfiguration()
{
	uint8_t data[2];
	HAL_I2C_Mem_Read(&hi2c1, INA219_DEFAULT_ADDR<<1, 0x00, 1, data, 2, 100);

	uint16_t config = (data[0]<<8)|data[1];

	return config;
}

double INA219getCurrent()
{
	uint8_t data[2];
	HAL_I2C_Mem_Read(&hi2c1, INA219_DEFAULT_ADDR<<1, 0x04, 1, data, 2, 100);

	int16_t currentReg = (int16_t)((data[0]<<8)|data[1]);

	double current = currentReg*currentLSB;
	return current;
}

double INA219getBusVoltage()
{
	uint8_t data[2];
	HAL_I2C_Mem_Read(&hi2c1, INA219_DEFAULT_ADDR<<1, 0x02, 1, data, 2, 100);

	uint16_t voltReg = ((data[0]<<8)|data[1])>>3;

	double busLSB = 0.004;
	double voltage = (voltReg*busLSB);
	return voltage;
}

double INA219getShuntVoltage()
{
	uint8_t data[2];
	HAL_I2C_Mem_Read(&hi2c1, INA219_DEFAULT_ADDR<<1, 0x01, 1, data, 2, 100);

	int16_t voltReg = (int16_t)((data[0]<<8)|data[1]);

	double shuntLSB = 0.00001;
	double voltage = voltReg*shuntLSB;
	return voltage;
}

double INA219getPower()
{
	uint8_t data[2];
	HAL_I2C_Mem_Read(&hi2c1, INA219_DEFAULT_ADDR<<1, 0x04, 1, data, 2, 100);

	uint16_t powerReg = (data[0]<<8)|data[1];

	double powerLSB = 20*currentLSB;
	double power = powerReg*(powerLSB);
	return power;
}
