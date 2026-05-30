#ifndef SENSOR_CALIBRATION_HPP
#define SENSOR_CALIBRATION_HPP

#include <cstdint>

// Enumeration for 6-axis accelerometer face calibration
enum class AccelFace {
    FACE_XP, // X Positive (+1g)
    FACE_XN, // X Negative (-1g)
    FACE_YP, // Y Positive (+1g)
    FACE_YN, // Y Negative (-1g)
    FACE_ZP, // Z Positive (+1g)
    FACE_ZN  // Z Negative (-1g)
};

// Calibration data structure per sensor type
struct CalibrationData {
    float biasX = 0.0f;
    float biasY = 0.0f;
    float biasZ = 0.0f;
    
    float rangeX = 0.0f;
    float rangeY = 0.0f;
    float rangeZ = 0.0f;
    
    bool isCalibrated = false;
};

class SensorCalibration {
public:
    SensorCalibration() = default;
    ~SensorCalibration() = default;

    /**
     * @brief Calibrates the gyroscope assuming it is perfectly static.
     */
    void calibrateGyro(const float* rawGyroX, const float* rawGyroY, 
                                      const float* rawGyroZ, uint16_t numSamples);


    /**
     * @brief Calibrates a single specific face of the accelerometer for multi-position calibration.
     */
    void calibrateAccelFace(const int16_t* rawAccelX, const int16_t* rawAccelY, 
                             const int16_t* rawAccelZ, uint16_t numSamples,
                             AccelFace face);

    AccelFace faceFSM(AccelFace currentFace);

    /**
     * @brief Calibrates the compass/magnetometer.
     */
    void calibrateCompass(const int16_t* rawMagX, const int16_t* rawMagY, 
                           const int16_t* rawMagZ, uint16_t numSamples);

    // Getters to fetch calibration matrices/vectors
    const CalibrationData& getGyroCalib() const { return gyroCalib; }
    const CalibrationData& getAccelCalib() const { return accelCalib; }
    const CalibrationData& getCompassCalib() const { return compassCalib; }

private:
    CalibrationData gyroCalib;
    CalibrationData accelCalib;
    CalibrationData compassCalib;
};

#endif // SENSOR_CALIBRATION_HPP