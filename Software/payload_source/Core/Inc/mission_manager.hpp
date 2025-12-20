/**
  ******************************************************************************
  * @file           : mission_manager.hpp
  * @author         : RSX 2025-2026
  * @brief          : Declares MissionManager class for ../Lib/mission_manager.cpp
  ******************************************************************************
  */

#ifndef INC_COMMAND_MANAGER_HPP_
#define INC_COMMAND_MANAGER_HPP_

#include "global_includes.h"

class MissionManager
{
private:

    OperatingState op_state = OperatingState::IDLE;
    SimModeStatus sim_status = SimModeStatus::SIM_OFF;
    OperatingMode op_mode = OperatingMode::OPMODE_FLIGHT;
    bool ALT_CAL_CHK = False;
    int packet_count = 0;
    float launch_altitude = 0.0;
    int SIMP_DATA = 0;
    bool waiting_for_simp = False;
    char* last_command;

public:

    OperatingState getOpState();

    SimModeStatus getSimStatus();

    OperatingMode getOpMode();

    void setOpState(OperatingState state);

    void setSimStatus(SimModeStatus status);

    void setOpMode(OperatingMode mode);

    bool isAltCalibrated();

    void setAltCalibration(float alt);

    float getLaunchAlt();

    void setAltCalOff();

    int getPacketCount();

    void incrPacketCount();

    void clearPacketCount();

    void waitingForSimp();

    void simpRecv();

    bool isWaitingSimp();

    void setSimpData(int data);

    int getSimpData();

    void setLastCommand(char* cmd);

    char* getLastCommand();
};

#endif /* INC_COMMAND_MANAGER_HPP_ */
