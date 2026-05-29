/**
  ******************************************************************************
  * @file           : mission_manager.hpp
  * @author         : RSX 2025-2026
  * @brief          : Declares MissionManager class for ../Lib/mission_manager.cpp
  ******************************************************************************
  */

#ifndef INC_MISSION_MANAGER_HPP
#define INC_MISSION_MANAGER_HPP

#include "global_includes.hpp"

class MissionManager
{
private:

    OperatingState op_state = OperatingState::IDLE;
    FlightCtrl flight_ctrl = FlightCtrl::AUTONOMOUS;
    SimModeStatus sim_status = SimModeStatus::SIM_OFF;
    OperatingMode op_mode = OperatingMode::OPMODE_FLIGHT;
    bool ALT_CAL_CHK = false;
    int packet_count = 0;
    float launch_altitude = 0.0;
    int SIMP_DATA = 0;
    bool waiting_for_simp = false;
    char last_command[CMD_BUFF_SIZE];
    bool logfile_chk = true;
    float alt_buffer[ALTITUDE_SMOOTHING_WINDOW] = {0};
    int alt_buffer_idx = 0;
    float max_alt = 0.0;
    int descent_trigger_count = 0;
    bool apogee_flag = false;
    bool nosecone_flag = false;
    bool probe_flag = false;
    bool wing_flag = false;
    bool egg_flag = false;
    int landed_trigger_count = 0;

public:

    bool nosecone_check();
    bool probe_check();
    bool egg_check();
    bool wing_check();
    void nosecone_rel();
    void probe_rel();
    void wing_rel();
    void egg_rel();

    bool landed_trigger();

    bool is_apogee_packet_sent();

    void apogee_packet_sent();

    void reset_params();

    bool descent_trigger();

    float get_max_alt();

    bool update_max_alt(float value);

    void update_alt_buffer(float value);

    float calculate_median_alt();

    OperatingState getOpState();

    FlightCtrl getFlightCtrl();

    SimModeStatus getSimStatus();

    OperatingMode getOpMode();

    void setOpState(OperatingState state);

    void setFlightCtrl(FlightCtrl ctrl);

    void setSimStatus(SimModeStatus status);

    void setOpMode(OperatingMode mode);

    bool isAltCalibrated();

    void setAltCalibration(float alt);

    float getLaunchAlt();

    void setPacketCount(int count);

    int getPacketCount();

    void incrPacketCount();

    void waitingForSimp();

    void simpRecv();

    bool isWaitingSimp();

    void setSimpData(int data);

    int getSimpData();

    void setLastCommand(char* cmd);

    char* getLastCommand();

    bool logfile_ok();

    void disableLogfile();

    void enableLogfile();
};

#endif /* INC_MISSION_MANAGER_HPP */
