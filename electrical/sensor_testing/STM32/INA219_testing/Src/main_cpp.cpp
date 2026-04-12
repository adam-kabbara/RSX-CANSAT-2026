#include "main.h"
extern I2C_HandleTypeDef hi2c1;

#define GPIOA ((GPIO_TypeDef *) GPIOA_BASE)

void setup()
{
	printf("Setup completed successfully.\n");
}

void loop()
{
	HAL_Delay(1000); //just in case

	//check to see if the device is connected properly
	printf("Waiting for device...\r\n");
	while(HAL_I2C_IsDeviceReady(&hi2c1, 0x40<<1, 1, 100) != 0x00)
	{
		BSP_LED_Toggle(LED_GREEN);
		HAL_Delay(100);
	}

	//if device is connected then do the testing
	printf("Found device\r\n");
	while(1)
	{
		// Toggle Board LED
		uint8_t data[2];
		if(HAL_I2C_Mem_Read(&hi2c1, 0x40<<1, 0x02, 1, data, 2, 100) == 0x00){
			uint16_t current = (data[0]<<8)|data[1];
			int bv = (int)(100*32.0*(float)current/(2<<16-1));
			printf("Bus Voltage: %d.%d\r\n",bv/100,bv%100);
		}
		else
		{
			printf("error.\r\n");
		}
		HAL_Delay(1000);
	}
}

extern "C" void main_cpp()
{
	setup();

	loop();
}


