/**
  ******************************************************************************
  * @file           : sensor_manager.h
  * @author         : RSX 2025-2026
  * @brief          : Declares SensorManager class for ../Lib/sensor_manager.cpp
  ******************************************************************************
  */

#ifndef INC_SENSOR_MANAGER_H_
#define INC_SENSOR_MANAGER_H_

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "global_includes.h"
#include "serialManager.h"
#include "missionManager.h"

class SensorManager
{
private:

    typedef struct transmission_packet {
        int TEAM_ID_PCKT = 0;
        char MISSION_TIME[DATA_SIZE] = "";
        int PACKET_COUNT = 0;
        char MODE[2] = "";
        char STATE[DATA_SIZE] = "";
        float ALTITUDE = 0.0;
        float TEMPERATURE = 0.0;
        float PRESSURE = 0.0;
        float VOLTAGE = 0.0;
        int GYRO_R = 0;
        int GYRO_P = 0;
        int GYRO_Y = 0;
        int ACCEL_R = 0;
        int ACCEL_P = 0;
        int ACCEL_Y = 0;
        char GPS_TIME[DATA_SIZE] = "";
        float GPS_ALTITUDE = 0.0;
        float GPS_LATITUDE = 0.0;
        float GPS_LONGITUDE = 0.0;
        int GPS_SATS = 0;
        char CMD_ECHO[CMD_BUFF_SIZE] = "";
        int CAMERA_STATUS = 0;
    } transmission_packet;

    transmission_packet send_packet;

    typedef struct altitude_data {
        size_t window_size = ALTITUDE_WINDOW_SIZE;
        float buffer[ALTITUDE_WINDOW_SIZE] = {0};
        int idx = 0;
        float current_alt = 0.0;
        float max_alt = 0.0;
        int sample_count = 0;
    } altitude_data;

    altitude_data alt_data;

public:

    SensorManager();

    void sampleSensors(MissionManager &mission_info, SerialManager &ser);

    void build_data_str(char *buff, size_t size);

    float pressure_to_alt(const float pressure);

    void cmd_buff_to_echo(char *cmd_buff);

    void resetAltData();

    float getPressure();

    float getTemp();

    void setAltData(float alt);

    void startSensors(SerialManager &ser, MissionManager &info);

    void writeServo(int servo_idx, int pos);

    void writeCameraServo(int pos);

    void getGpsAlt(float *alt, float launch_alt);

    void getGpsLat(float *lat);

    void getGpsLong(float *lon);

    void getGpsSats(int *sat);

    void getRtcTime(char time_str[DATA_SIZE]);

    void getGpsTime(char time_str[DATA_SIZE]);

    float getVoltage();

    float getCurrent();

    void setRtcTime(int sec, int minute, int hour);

    int getCamera1Status();

    int getCamera2Status();

    void updateCameraServo();

    void update_imu();
};

#endif /* SENSOR_MANAGER_H */

#endif /* INC_SENSOR_MANAGER_H_ */
