#ifndef MISSION_H
#define MISSION_H

//#include "includes.h"
#include <serial_manager.hpp>
//#include <Preferences.h>

class MissionManager
{
private:
    Preferences preferences;

    typedef struct mission_info_struct {
        OperatingState op_state = IDLE;
        SimModeStatus sim_status = SIM_OFF;
        OperatingMode op_mode = OPMODE_FLIGHT;
        uint8_t ALT_CAL_CHK = 0;
        int packet_count = 0;
        float launch_altitude = 0.0;
        int SIMP_DATA = 0;
        uint8_t waiting_for_simp = 0;
    } mission_info_struct;

    mission_info_struct mission_info;

public:

    OperatingState getOpState();

    SimModeStatus getSimStatus();

    OperatingMode getOpMode();

    void setOpState(OperatingState state);

    void setSimStatus(SimModeStatus status);

    void setOpMode(OperatingMode mode);

    uint8_t isAltCalibrated();

    void setAltCalibration(float alt);

    float getLaunchAlt();

    void setAltCalOff();

    int getPacketCount();

    void incrPacketCount();

    void clearPacketCount();

    void waitingForSimp();

    void simpRecv();

    uint8_t isWaitingSimp();

    void setSimpData(int data);

    int getSimpData();

    void beginPref(const char *name, bool rw);

    void endPref();

    void putPrefInt(const char *key, int value);

    int getPrefInt(const char *key, int def);

    void putPrefFloat(const char *key, float value);

    float getPrefFloat(const char *key, float def);

    void clearPref();

    void resetSeq(SerialManager &serial);
};

#endif /* MISSION_H */
