/**
  ******************************************************************************
  * @file           : command_manager.hpp
  * @author         : RSX 2025-2026
  * @brief          : Declares CommandManager class for ../Lib/command_manager.cpp
  ******************************************************************************
  */

#ifndef INC_COMMAND_MANAGER_HPP
#define INC_COMMAND_MANAGER_HPP

#include "global_includes.hpp"
#include "serial_manager.hpp"
#include "mission_manager.hpp"
#include "sensor_manager.hpp"

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
    void do_cal2(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_ctrl(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_gps(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);

    void status_update(SerialManager &ser, MissionManager &info);

public:
    CommandManager();

    uint8_t processCommand(const char *cmd_buff, SerialManager &ser, MissionManager &info, SensorManager &sensors);
};


#endif /* INC_COMMAND_MANAGER_HPP */
