#include "main.h"
#include "minmea.h"
#include "m10s.h"





void setup()
{
	printf("Setup completed successfully.\n");
	blocking_gps_i2c_verify();
}

void loop()
{
	while(1)
	{
		printf("\rChecking...\n");
		auto frame_rmc = get_gps_data_rmc(1);
		if(data_is_valid())
		{
			printf("\rSpeed: %f\n", frame_rmc.speed);
		}
		auto frame_gga = get_gps_data_gga(1);
		if(data_is_valid()){
			printf("\rFQ: %f\n", frame_gga.fix_quality);
		}
	}
}

extern "C" void main_cpp()
{
	setup();

	loop();
}


