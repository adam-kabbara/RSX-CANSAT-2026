/*
 * BMP581 pressure sensor I2C driver
 *
 * Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf
 *
 */

#ifndef INC_BMP581_H_
#define INC_BMP581_H_

#include <stm32WBAxx_hal.h>

// Register addresses
#define BMP5_REG_CHIP_ID			0x01
#define BMP5_REG_REV_ID				0x02
#define BMP5_REG_CHIP_STATUS		0x11
#define BMP5_REG_DRIVE_CONFIG 		0x13
#define BMP5_REG_INT_CONFIG			0x14
#define BMP5_REG_INT_SOURCE			0x15
#define BMP5_REG_FIFO_CONFIG		0x16
#define BMP5_REG_FIFO_COUNT			0x17
#define BMP5_REG_FIFO_SEL			0x18
#define BMP5_REG_TEMP_DATA_XLSB		0x1D
#define BMP5_REG_TEMP_DATA_LSB		0x1E
#define BMP5_REG_TEMP_DATA_MSB		0x1F
#define BMP5_REG_PRESS_DATA_XLSB	0x20
#define BMP5_REG_PRESS_DATA_LSB		0x21
#define BMP5_REG_PRESS_DATA_MSB		0x22
#define BMP5_REG_INT_STATUS			0x27
#define BMP5_REG_STATUS				0x28
#define BMP5_REG_FIFO_DATA			0x29
#define BMP5_REG_NVM_ADDR			0x2B
#define BMP5_REG_NVM_DATA_LSB		0x2C
#define BMP5_REG_NVM_DATA_MSB		0x2D
#define BMP5_REG_DSP_CONFIG			0x30
#define BMP5_REG_DSP_IIR			0x31
#define BMP5_REG_OOR_THR_P_LSB		0x32
#define BMP5_REG_OOR_THR_P_MSB		0x33
#define BMP5_REG_OOR_RANGE			0x34
#define BMP5_REG_OOR_CONFIG			0x35
#define BMP5_REG_OSR_CONFIG			0x36
#define BMP5_REG_ODR_CONFIG			0x37
#define BMP5_REG_OSR_EFF			0x38
#define BMP5_REG_CMD				0x7E

// I2C addresses
#define BMP5_I2C_ADDR_FIRST			0x47 // (SDO pulled down)
#define BMP5_I2C_ADDR_SECOND		0x46 // (SDO pulled up)

// CMD commands
#define BMP5_SOFT_RESET_CMD			0xB6
#define BMP5_NVM_FIRST_CMD			0x5D
#define BMP5_NVM_READ_ENABLE_CMD	0xA5
#define BMP5_NVM_WRITE_ENABLE_CMD	0xA0

// I2C communication timeout
#define BMP5_TIMEOUT_MS       		10

// Register values
#define BMP5_IIR_BYPASS				0x0
#define BMP5_IIR_COEF_1				0x1
#define BMP5_IIR_COEF_3				0x2
#define BMP5_IIR_COEF_7				0x3
#define BMP5_IIR_COEF_15			0x4
#define BMP5_IIR_COEF_31			0x5
#define BMP5_IIR_COEF_63			0x6
#define BMP5_IIR_COEF_127			0x7

#define BMP5_OSR_X1					0x0
#define BMP5_OSR_X2					0x1
#define BMP5_OSR_X4					0x2
#define BMP5_OSR_X8					0x3
#define BMP5_OSR_X16				0x4
#define BMP5_OSR_X32				0x5
#define BMP5_OSR_X64				0x6
#define BMP5_OSR_X128				0x7

#define BMP5_MODE_STANDBY			0x0
#define BMP5_MODE_NORMAL			0x1
#define BMP5_MNODE_FORCED			0x2
#define BMP5_MODE_CONTINIOUS		0x3

#define BMP5_ODR_240HZ				0x0
#define BMP5_OSR_218_537HZ			0x1
#define BMP5_OSR_199_111HZ			0x2
#define BMP5_OSR_179_200HZ			0x3
#define BMP5_OSR_160HZ				0x4
#define BMP5_OSR_149_333HZ			0x5
#define BMP5_OSR_140HZ				0x6
#define BMP5_OSR_129_855HZ			0x7
#define BMP5_ODR_120HZ				0x8
#define BMP5_OSR_110_164HZ			0x9
#define BMP5_OSR_100_299HZ			0xA
#define BMP5_OSR_89_600HZ			0xB
#define BMP5_OSR_80HZ				0xC
#define BMP5_OSR_70HZ				0xD
#define BMP5_OSR_60HZ				0xE
#define BMP5_OSR_50_056HZ			0xF
#define BMP5_ODR_45_025HZ			0x10
#define BMP5_OSR_40HZ				0x11
#define BMP5_OSR_35HZ				0x12
#define BMP5_OSR_30HZ				0x13
#define BMP5_OSR_25HZ				0x14
#define BMP5_OSR_20HZ				0x15
#define BMP5_OSR_15HZ				0x16
#define BMP5_OSR_10HZ				0x17
#define BMP5_ODR_5HZ				0x18
#define BMP5_OSR_4HZ				0x19
#define BMP5_OSR_3HZ				0x1A
#define BMP5_OSR_2HZ				0x1B
#define BMP5_OSR_1HZ				0x1C
#define BMP5_OSR_0_500HZ			0x1D
#define BMP5_OSR_0_250HZ			0x1E
#define BMP5_OSR_0_125HZ			0x1F



typedef struct {
	I2C_HandleTypeDef *i2cHandle;
	uint8_t i2c_addr;

	uint8_t dataReady;

	float temperature;
	float pressure;

} BMP5;

uint8_t BMP5_Init(BMP5 *dev, I2C_HandleTypeDef *i2cHandle, uint8_t addr);
uint8_t BMP5_SOFT_DataReady(BMP5 *dev);
uint8_t BMP5_SaveConvData(BMP5 *dev);
void BMP5_GetConvData(BMP5 *dev, float *pressure, float *temperature);
uint8_t BMP5_Start_Mode(BMP5 *dev, uint8_t mode, uint8_t dataRate, uint8_t osr_p, uint8_t osr_t);
uint8_t BMP5_Set_IIR(BMP5 *dev, uint8_t iir_p, uint8_t iir_t);
uint8_t BMP5_Sleep_Mode(BMP5 *dev);
uint8_t BMP5_SOFT_Reset(BMP5 *dev);


uint8_t BMP5_WriteReg(BMP5 *dev, uint8_t reg, uint8_t size, uint8_t *data);
uint8_t BMP5_ReadReg(BMP5 *dev, uint8_t reg, uint8_t size, uint8_t *data);

#endif
