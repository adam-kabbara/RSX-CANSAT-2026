#include "main.h"

void setup()
{
	printf("Setup completed successfully.\n");
}

void loop()
{
	while(1)
	{
		// Toggle Board LED
		printf("Hello there!\n");
		BSP_LED_Toggle(LED_GREEN);
		HAL_Delay(1000);
	}
}

extern "C" void main_cpp()
{
	setup();

	loop();
}


