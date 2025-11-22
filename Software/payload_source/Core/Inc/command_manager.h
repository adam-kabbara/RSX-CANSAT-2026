/**
  ******************************************************************************
  * @file           : command_manager.h
  * @author         : RSX 2025-2026
  * @brief          : Declares CommandManager class for ../Lib/command_manager.cpp
  ******************************************************************************
  */

#ifndef INC_COMMAND_MANAGER_H_
#define INC_COMMAND_MANAGER_H_

#include <global_includes.hpp>
#include <serial_manager.hpp>
#include "missionManager.h"
#include "sensorManager.h"

class CommandManager {
private:

    std::unordered_map<std::string, std::function<void(SerialManager&, MissionManager&, SensorManager&, const char*)>> command_map;

    // Command processing functions
    void do_cx(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_st(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_restart(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_give_status(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_sim(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_simp(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_cal(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_mec(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_logs(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);

public:
    CommandManager();

    int processCommand(const char *cmd_buff, SerialManager &ser, MissionManager &info, SensorManager &sensors);
};


#endif /* INC_COMMAND_MANAGER_H_ */
