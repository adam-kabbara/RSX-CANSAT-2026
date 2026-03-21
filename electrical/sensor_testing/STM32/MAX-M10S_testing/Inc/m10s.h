/*
 * m10s.h
 *
 *  Created on: Mar 21, 2026
 *      Author: Hammy
 */

#ifndef INC_M10S_H_
#define INC_M10S_H_

#include "minmea.h"

bool data_is_valid();
void blocking_gps_i2c_verify();
struct minmea_sentence_rmc get_gps_data_rmc(int attempts); //Returns first set of rmc data in nmea stream (lat/lon/speed). Check validity with data_valid (I'm sorry).
struct minmea_sentence_gga get_gps_data_gga(int attempts); //Returns first set of gga data in nmea stream (high-precision data + altitude). Check validity with data_valid (I'm sorry, again).


#endif /* INC_M10S_H_ */
