#include "sensor_calibration.hpp"
#include <algorithm>
#include <cmath>

// Gyroscope calibration - Simple zero-bias average (Assumes sensor is perfectly still)
void SensorCalibration::calibrateGyro(const float* rawGyroX, const float* rawGyroY, 
                                      const float* rawGyroZ, uint16_t numSamples) {
    if (numSamples == 0) return;

    double sumX = 0, sumY = 0, sumZ = 0;
    
    for (uint16_t i = 0; i < numSamples; i++) {
        sumX += rawGyroX[i];
        sumY += rawGyroY[i];
        sumZ += rawGyroZ[i];
    }
    
    gyroCalib.biasX = static_cast<float>(sumX / numSamples);
    gyroCalib.biasY = static_cast<float>(sumY / numSamples);
    gyroCalib.biasZ = static_cast<float>(sumZ / numSamples);
    gyroCalib.isCalibrated = true;
}


AccelFace SensorCalibration::faceFSM(AccelFace currentFace) {
    switch (currentFace) {
        case AccelFace::FACE_XP: return AccelFace::FACE_XN;
        case AccelFace::FACE_XN: return AccelFace::FACE_YP;
        case AccelFace::FACE_YP: return AccelFace::FACE_YN;
        case AccelFace::FACE_YN: return AccelFace::FACE_ZP;
        case AccelFace::FACE_ZP: return AccelFace::FACE_ZN;
        case AccelFace::FACE_ZN: return AccelFace::FACE_XP;
        default: return AccelFace::FACE_XP; // Default to start if invalid
    }
}

// Accelerometer multi-face calibration (Calculates true offset/scale via max/min tracking)
void SensorCalibration::calibrateAccelFace(const int16_t* rawAccelX, const int16_t* rawAccelY, 
                                           const int16_t* rawAccelZ, uint16_t numSamples,
                                           AccelFace face) {
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
        case AccelFace::FACE_XP: accelCalib.rangeX = meanX; break;
        case AccelFace::FACE_XN: accelCalib.biasX  = meanX; break;
        case AccelFace::FACE_YP: accelCalib.rangeY = meanY; break;
        case AccelFace::FACE_YN: accelCalib.biasY  = meanY; break;
        case AccelFace::FACE_ZP: accelCalib.rangeZ = meanZ; break;
        case AccelFace::FACE_ZN: accelCalib.biasZ  = meanZ; break;
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
void SensorCalibration::calibrateCompass(const int16_t* rawMagX, const int16_t* rawMagY, 
                                         const int16_t* rawMagZ, uint16_t numSamples) {
    if (numSamples == 0) return;
    
    int16_t minX = rawMagX[0], maxX = rawMagX[0];
    int16_t minY = rawMagY[0], maxY = rawMagY[0];
    int16_t minZ = rawMagZ[0], maxZ = rawMagZ[0];
    
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