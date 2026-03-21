#include "main.h"
#include "minmea.h"

#define M10S_ADDR (0x42<<1)

extern I2C_HandleTypeDef hi2c1;

void verify_i2c()
{
	BSP_LED_Off(LED_GREEN);
	while(HAL_I2C_IsDeviceReady(&hi2c1, M10S_ADDR,1, 100) != HAL_OK)
	{
		BSP_LED_Toggle(LED_GREEN);
		HAL_Delay(100);
	}
	BSP_LED_On(LED_GREEN);
}

void setup()
{
	printf("Setup completed successfully.\n");
	verify_i2c();
}

void loop()
{
	while(1)
	{
		// Toggle Board LED
		uint8_t avail[2] = {0, 0};
		uint16_t toread = 0;
		int cache_size = MINMEA_MAX_SENTENCE_LENGTH;
		char data[cache_size]; int d = 0;
		data[cache_size-1] = '\0';

		HAL_I2C_Mem_Read(&hi2c1, M10S_ADDR, 0xFD, 1, avail, 1, 100);
		HAL_I2C_Mem_Read(&hi2c1, M10S_ADDR, 0xFE, 1, (avail+1), 1, 100);
		toread = *avail;

		printf("\rStream Size: %d\n", toread);
		HAL_Delay(100);

		if(toread == 0){continue;}

		while(toread > 0 && d < cache_size-1)
		{
			if(HAL_I2C_Mem_Read(&hi2c1, M10S_ADDR, 0xFF, 1, (uint8_t*)(data)+d, 1, 100) != HAL_OK){break;}

			if(data[d] == '\n'){break;}
			d++;

			if(HAL_I2C_Mem_Read(&hi2c1, M10S_ADDR, 0xFD, 1, avail, 1, 100) != HAL_OK){break;}
			if(HAL_I2C_Mem_Read(&hi2c1, M10S_ADDR, 0xFE, 1, (avail+1), 1, 100) != HAL_OK){break;}
			toread = *avail;

			//printf("\rData: %c\t\t To Read: %d\n", data[d-1], toread);

		}

		printf("\r%s\n", data);


		//BSP_LED_Toggle(LED_GREEN);
		auto state = minmea_sentence_id(data, false);
		switch (state) {
		        case MINMEA_SENTENCE_RMC: {
		            struct minmea_sentence_rmc frame;
		            if (minmea_parse_rmc(&frame, data)) {
		                printf("\r$RMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
		                        frame.latitude.value, frame.latitude.scale,
		                        frame.longitude.value, frame.longitude.scale,
		                        frame.speed.value, frame.speed.scale);
		                printf("\r$RMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
		                        minmea_rescale(&frame.latitude, 1000),
		                        minmea_rescale(&frame.longitude, 1000),
		                        minmea_rescale(&frame.speed, 1000));
		                double lat = minmea_tocoord(&frame.latitude);
		                double lon = minmea_tocoord(&frame.longitude);
		                printf("\r$RMC floating point degree coordinates and speed: (%f,%f) %f\n",minmea_tocoord(&frame.latitude),minmea_tocoord(&frame.longitude), minmea_tofloat(&frame.speed));
		            }
		        } break;

		        case MINMEA_SENTENCE_GGA: {
		            struct minmea_sentence_gga frame;
		            if (minmea_parse_gga(&frame, data)) {
		                printf("\r$GGA: fix quality: %d\n", frame.fix_quality);
		            }
		        } break;

		        case MINMEA_SENTENCE_GSV: {
		            struct minmea_sentence_gsv frame;
		            if (minmea_parse_gsv(&frame, data)) {
		                printf("\r$GSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
		                printf("\r$GSV: satellites in view: %d\n", frame.total_sats);
		                for (int i = 0; i < 4; i++)
		                    printf("\r$GSV: sat nr %d, elevation: %d, azimuth: %d, snr: %f dbm\n",frame.sats[i].nr,frame.sats[i].elevation,frame.sats[i].azimuth,minmea_tofloat(&frame.sats[i].snr));
		            }
		        } break;

		        default: {
		        	printf("Locking...");
		        }
		  }
	}
}

extern "C" void main_cpp()
{
	setup();

	loop();
}


