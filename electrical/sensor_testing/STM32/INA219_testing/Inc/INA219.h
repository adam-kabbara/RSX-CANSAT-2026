/*
 * INA219.h
 *
 *  Created on: May 2, 2026
 *      Author: Kenneth Freeman
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

/**
  * @brief Sets up connection to INA219 and resets calibration register, returns false on a failure
  */
bool INA219setup(double maxCurrent, double shuntResistance = 0.1, short unsigned int overwriteCalibrationValue = 0);


/**
  * @brief Writes bitwise configuration (refer to INA219 datasheet for each value)
  */
void INA219overwriteConfiguration(short unsigned int configurationValue);

/**
  * @brief Returns configuration value in INA219 configuration register (refer to INA219 datasheet for each value)
  */
short unsigned int INA219getConfiguration();

/**
  * @brief Returns actual current value through shunt resistor based on value from ina219 current register
  */
double INA219getCurrent();

/**
  * @brief Returns actual voltage across shunt resistor on value from ina219 shunt voltage register
  */
double INA219getShuntVoltage();

/**
  * @brief Returns actual bus voltage based on value from ina219 bus voltage register
  */
double INA219getBusVoltage();

/**
  * @brief Returns actual bus power based on value from ina219 bus voltage register
  */
double INA219getPower();



#endif /* INC_INA219_H_ */
