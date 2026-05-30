#ifndef SENSOR_CALIBRATION_HPP
#define SENSOR_CALIBRATION_HPP

#include <cstdint>
#include "sensor_manager.hpp"

#define READINGCOUNT 200 //Maximum number of readings per sensor

// Enumeration for 1-stage magnetometer/gyro calibration + 6-axis accelerometer face calibration
enum class CalibrationState {
	IDLE,		// Nothing
	MAG_RUN, 	// Magnetometer
	GYRO_RUN, 	// Gyro
    FACE_XP, 	// X Positive (+1g)
    FACE_XN, 	// X Negative (-1g)
    FACE_YP, 	// Y Positive (+1g)
    FACE_YN, 	// Y Negative (-1g)
    FACE_ZP, 	// Z Positive (+1g)
    FACE_ZN  	// Z Negative (-1g)
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
	 * @brief Sets all elements in the calibration readings array to 0.
	 */
    void resetReadings();

    /**
	 * @brief Returns X-axis calibration readings array.
	 */
	float* getReadingsX();

	/**
	* @brief Returns Y-axis calibration readings array.
	*/
	float* getReadingsY();

	/**
	* @brief Returns Z-axis calibration readings array.
	*/
	float* getReadingsZ();

	/**
	 * @brief Returns current number of readings.
	 */
	int16_t getReadingsCount();

    /**
	 * @brief Collects 1 instance of sensor data required to calibrate the sensor corresponding to the current calibration state.
	 */
    void collectReading(CalibrationState calibrationState, SensorManager sensors);

    /**
     * @brief Calibrates the gyroscope assuming it is perfectly static.
     */
    void calibrateGyro(const float* rawGyroX, const float* rawGyroY,
                        const float* rawGyroZ, uint16_t numSamples);


    /**
     * @brief Calibrates a single specific face of the accelerometer for multi-position calibration.
     */
    void calibrateAccelFace(const float* rawAccelX, const float* rawAccelY,
                             const float* rawAccelZ, uint16_t numSamples,
                             CalibrationState face);

    static CalibrationState calibrationFSM(CalibrationState currentCalibrationState);

    /**
     * @brief Calibrates the compass/magnetometer.
     */
    void calibrateCompass(const float* rawMagX, const float* rawMagY,
                           const float* rawMagZ, uint16_t numSamples);

    // Getters to fetch calibration matrices/vectors
    const CalibrationData& getGyroCalib() const { return gyroCalib; }
    const CalibrationData& getAccelCalib() const { return accelCalib; }
    const CalibrationData& getCompassCalib() const { return compassCalib; }

private:
    /**
	 * @brief Increments the reading counter and the reading index.
	 */
    void incrementReadingCount();
    CalibrationData gyroCalib;
    CalibrationData accelCalib;
    CalibrationData compassCalib;
    int16_t currentReadings;
    int16_t readingIndex;
    float calibrationReadingsX[READINGCOUNT];
    float calibrationReadingsY[READINGCOUNT];
    float calibrationReadingsZ[READINGCOUNT];
};

#endif // SENSOR_CALIBRATION_HPP
