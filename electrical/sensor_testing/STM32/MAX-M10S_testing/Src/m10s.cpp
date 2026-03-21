#include "minmea.h"
#include "m10s.h"
#include "main.h"

#define M10S_ADDR (0x42<<1)

extern I2C_HandleTypeDef hi2c1;
static bool data_valid = false;

bool data_is_valid() //Returns running check of validity of last sent data
{
	return data_valid;
}

void blocking_gps_i2c_verify() //waits until i2c works properly for gps
{
	BSP_LED_Off(LED_GREEN);
	while(HAL_I2C_IsDeviceReady(&hi2c1, M10S_ADDR,1, 100) != HAL_OK)
	{
		BSP_LED_Toggle(LED_GREEN);
		HAL_Delay(100);
	}
	BSP_LED_On(LED_GREEN);
}

minmea_sentence_rmc get_gps_data_rmc(int attempts) //Returns first set of rmc data in nmea stream (lat/lon/speed). Check validity with data_valid (I'm sorry).
{
	for(int attempt = 0; attempt < attempts; attempt++){
		int cache_size = MINMEA_MAX_SENTENCE_LENGTH;
		char data[cache_size]; int d = 0; //contains data stream
		data[cache_size-1] = '\0';

		//read data stream from gps
		while(d < cache_size-1)
		{
			if(HAL_I2C_Mem_Read(&hi2c1, M10S_ADDR, 0xFF, 1, (uint8_t*)(data)+d, 1, 100) != HAL_OK){break;}
			if(data[d] == '\n'){break;}
			d++;
		}

		//process data stream
		auto state = minmea_sentence_id(data, false);
		if(state == MINMEA_SENTENCE_RMC)
		{

			struct minmea_sentence_rmc frame;
			if (minmea_parse_rmc(&frame, data)) {
				data_valid = true;
				return frame;
			}
		}
	}
	minmea_sentence_rmc null_frame;
	data_valid = false;
	return null_frame;
}

minmea_sentence_gga get_gps_data_gga(int attempts) //Returns first set of gga data in nmea stream (high-precision data + altitude). Check validity with data_valid (I'm sorry, again).
{
	for(int attempt = 0; attempt < attempts; attempt++){
		int cache_size = MINMEA_MAX_SENTENCE_LENGTH;
		char data[cache_size]; int d = 0; //contains data stream
		data[cache_size-1] = '\0';

		//read data stream from gps
		while(d < cache_size-1)
		{
			if(HAL_I2C_Mem_Read(&hi2c1, M10S_ADDR, 0xFF, 1, (uint8_t*)(data)+d, 1, 100) != HAL_OK){break;}
			if(data[d] == '\n'){break;}
			d++;
		}

		//process data stream
		auto state = minmea_sentence_id(data, false);
		if(state == MINMEA_SENTENCE_GGA)
		{
			struct minmea_sentence_gga frame;
			if (minmea_parse_gga(&frame, data)) {
				data_valid = true;
				return frame;
			}
		}
	}

	minmea_sentence_gga null_frame;
	data_valid = false;
	return null_frame;
}
