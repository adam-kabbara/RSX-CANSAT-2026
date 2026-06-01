#include "sensor_calibration.hpp"
#include "sensor_manager.hpp"
#include <algorithm>
#include <cmath>

// Gyroscope calibration - Simple zero-bias average (Assumes sensor is perfectly still)
void SensorCalibration::calibrateGyro(const float* rawGyroX, const float* rawGyroY,
                                      const float* rawGyroZ, uint16_t numSamples) {
    if (numSamples == 0) return;
    
    double sumX = 0, sumY = 0, sumZ = 0;
    float minX = rawGyroX[0], maxX = rawGyroX[0];
    float minY = rawGyroY[0], maxY = rawGyroY[0];
    float minZ = rawGyroZ[0], maxZ = rawGyroZ[0];
    
    for (uint16_t i = 0; i < numSamples; i++) {
        sumX += rawGyroX[i];
        sumY += rawGyroY[i];
        sumZ += rawGyroZ[i];
        
        minX = std::min(minX, rawGyroX[i]);
        maxX = std::max(maxX, rawGyroX[i]);
        minY = std::min(minY, rawGyroY[i]);
        maxY = std::max(maxY, rawGyroY[i]);
        minZ = std::min(minZ, rawGyroZ[i]);
        maxZ = std::max(maxZ, rawGyroZ[i]);
    }
    
    gyroCalib.biasX = static_cast<float>(sumX / numSamples);
    gyroCalib.biasY = static_cast<float>(sumY / numSamples);
    gyroCalib.biasZ = static_cast<float>(sumZ / numSamples);
    
    gyroCalib.rangeX = static_cast<float>(maxX - minX);
    gyroCalib.rangeY = static_cast<float>(maxY - minY);
    gyroCalib.rangeZ = static_cast<float>(maxZ - minZ);
    
    gyroCalib.isCalibrated = true;
}


CalibrationState SensorCalibration::calibrationFSM(CalibrationState currentFace) {
    switch (currentFace) {
    	case CalibrationState::MAG_RUN: 	return CalibrationState::IDLE;
    	case CalibrationState::GYRO_RUN:	return CalibrationState::IDLE;
        case CalibrationState::FACE_XP: 	return CalibrationState::FACE_XN;
        case CalibrationState::FACE_XN: 	return CalibrationState::FACE_YP;
        case CalibrationState::FACE_YP: 	return CalibrationState::FACE_YN;
        case CalibrationState::FACE_YN: 	return CalibrationState::FACE_ZP;
        case CalibrationState::FACE_ZP: 	return CalibrationState::FACE_ZN;
        case CalibrationState::FACE_ZN: 	return CalibrationState::IDLE;
        default: return CalibrationState::IDLE; // Default to start if invalid
    }
}

// Accelerometer multi-face calibration (Calculates true offset/scale via max/min tracking)
void SensorCalibration::calibrateAccelFace(const float* rawAccelX, const float* rawAccelY,
                                           const float* rawAccelZ, uint16_t numSamples,
                                           CalibrationState face) {
    if (numSamples == 0) return;
    
    double sumX = 0, sumY = 0, sumZ = 0;
    for (uint16_t i = 0; i < numSamples; i++) {
        sumX += rawAccelX[i];
        sumY += rawAccelY[i];
        sumZ += rawAccelZ[i];
    }
    
    float meanX = static_cast<float>(sumX / numSamples);
    float meanY = static_cast<float>(sumY / numSamples);
    float meanZ = static_cast<float>(sumZ / numSamples);
    
    // Internal helper uses range fields to store maximums and minimums temporarily
    // rangeX/Y/Z = positive max, biasX/Y/Z = negative min
    switch (face) {
        case CalibrationState::FACE_XP: accelCalib.rangeX = meanX; break;
        case CalibrationState::FACE_XN: accelCalib.biasX  = meanX; break;
        case CalibrationState::FACE_YP: accelCalib.rangeY = meanY; break;
        case CalibrationState::FACE_YN: accelCalib.biasY  = meanY; break;
        case CalibrationState::FACE_ZP: accelCalib.rangeZ = meanZ; break;
        case CalibrationState::FACE_ZN: accelCalib.biasZ  = meanZ; break;
        default: return; //quit if state is invalid
    }
    
    // Compute final combined parameters using min/max bounds
    float finalBiasX = (accelCalib.rangeX + accelCalib.biasX) / 2.0f;
    float finalBiasY = (accelCalib.rangeY + accelCalib.biasY) / 2.0f;
    float finalBiasZ = (accelCalib.rangeZ + accelCalib.biasZ) / 2.0f;
    
    float finalRangeX = accelCalib.rangeX - accelCalib.biasX;
    float finalRangeY = accelCalib.rangeY - accelCalib.biasY;
    float finalRangeZ = accelCalib.rangeZ - accelCalib.biasZ;

    // Overwrite with clean parameters
    accelCalib.biasX = finalBiasX;
    accelCalib.biasY = finalBiasY;
    accelCalib.biasZ = finalBiasZ;
    
    accelCalib.rangeX = finalRangeX;
    accelCalib.rangeY = finalRangeY;
    accelCalib.rangeZ = finalRangeZ;
    
    accelCalib.isCalibrated = true;
}

// Compass (Magnetometer) calibration - Uses Midpoint Envelope & Soft-Iron Compensation
void SensorCalibration::calibrateCompass(const float* rawMagX, const float* rawMagY,
                                         const float* rawMagZ, uint16_t numSamples) {
    if (numSamples == 0) return;
    
    float minX = rawMagX[0], maxX = rawMagX[0];
    float minY = rawMagY[0], maxY = rawMagY[0];
    float minZ = rawMagZ[0], maxZ = rawMagZ[0];
    
    for (uint16_t i = 0; i < numSamples; i++) {
        minX = std::min(minX, rawMagX[i]);
        maxX = std::max(maxX, rawMagX[i]);
        minY = std::min(minY, rawMagY[i]);
        maxY = std::max(maxY, rawMagY[i]);
        minZ = std::min(minZ, rawMagZ[i]);
        maxZ = std::max(maxZ, rawMagZ[i]);
    }
    
    // Correct Hard-Iron Bias: Midpoint of min/max values
    compassCalib.biasX = (maxX + minX) / 2.0f;
    compassCalib.biasY = (maxY + minY) / 2.0f;
    compassCalib.biasZ = (maxZ + minZ) / 2.0f;
    
    // Calculate Delta spans per axis
    float deltaX = maxX - minX;
    float deltaY = maxY - minY;
    float deltaZ = maxZ - minZ;
    
    // Average delta/radius used to standardize scale factors
    float avgDelta = (deltaX + deltaY + deltaZ) / 3.0f;
    
    // Soft-Iron compensation factors (stored in fields for ranges)
    // If an axis is squished, its scale factor scales it up to match the average
    compassCalib.rangeX = (deltaX != 0.0f) ? (avgDelta / deltaX) : 1.0f;
    compassCalib.rangeY = (deltaY != 0.0f) ? (avgDelta / deltaY) : 1.0f;
    compassCalib.rangeZ = (deltaZ != 0.0f) ? (avgDelta / deltaZ) : 1.0f;
    
    compassCalib.isCalibrated = true;
}

void SensorCalibration::resetReadings()
{
	currentReadings = 0; 	//set number of readings to 0
	readingIndex = 0;		//ensure new readings start from the first index
	//set all readings to 0
	for(int readingIndex = 0; readingIndex < READINGCOUNT; readingIndex++)
	{
		calibrationReadingsX[readingIndex] = 0;
		calibrationReadingsY[readingIndex] = 0;
		calibrationReadingsZ[readingIndex] = 0;
	}
}

int16_t SensorCalibration::getReadingsCount()
{
	return currentReadings;
}

void SensorCalibration::incrementReadingCount()
{
	//increment counters
	currentReadings++;
	readingIndex = (readingIndex + 1)%READINGCOUNT;
	//limit currentReadings to array size
	if(currentReadings > READINGCOUNT)
	{
		currentReadings = READINGCOUNT;
	}
}

void SensorCalibration::collectReading(CalibrationState calibrationState, SensorManager sensors)
{
	switch(calibrationState)
	{
		case CalibrationState::IDLE: break; //do nothing
		case CalibrationState::MAG_RUN: {//get magnetometer readings
            if (readingIndex == READINGCOUNT) {break;}
			float *magData = new float[3];
			sensors.getRawMag(magData);
			calibrationReadingsX[readingIndex] = magData[0];
			calibrationReadingsY[readingIndex] = magData[1];
			calibrationReadingsZ[readingIndex] = magData[2];
			incrementReadingCount();
            delete[] magData;
			break;
		}
		case CalibrationState::GYRO_RUN: {//get gyro readings
            if (readingIndex == READINGCOUNT) {break;}
			float *gyroData = new float[3];
			sensors.getRawGyro(gyroData);
			calibrationReadingsX[readingIndex] = gyroData[0];
			calibrationReadingsY[readingIndex] = gyroData[1];
			calibrationReadingsZ[readingIndex] = gyroData[2];
			incrementReadingCount();
            delete[] gyroData;
			break;
		}
		default: {//get accelerometer readings
            if (readingIndex == READINGCOUNT) {break;}
			float *accelData = new float[3];
			sensors.getRawAccel(accelData);
			calibrationReadingsX[readingIndex] = accelData[0];
			calibrationReadingsY[readingIndex] = accelData[1];
			calibrationReadingsZ[readingIndex] = accelData[2];
			incrementReadingCount();
            delete[] accelData;
			break;
        }
	}
}

float* SensorCalibration::getReadingsX()
{
	return calibrationReadingsX;
}

float* SensorCalibration::getReadingsY()
{
	return calibrationReadingsY;
}

float* SensorCalibration::getReadingsZ()
{
	return calibrationReadingsZ;
}

SensorCalibration calibrator; //global instance of the calibration class
CalibrationState cal_state = CalibrationState::IDLE; //global variable to track current face/state