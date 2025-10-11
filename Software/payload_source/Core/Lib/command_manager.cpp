/**
  ******************************************************************************
  * @file           : command_manager.c
  * @author         : RSX 2025-2026
  * @brief          : Processes commands received by ground station
  ******************************************************************************
  */

#include "command_manager.h"

CommandManager::CommandManager()
{
    // Initialize the map with corresponding functions
    using namespace std::placeholders;
    command_map.emplace("CX", std::bind(&CommandManager::do_cx, this, _1, _2, _3, _4));
    command_map.emplace("ST", std::bind(&CommandManager::do_st, this, _1, _2, _3, _4));
    command_map.emplace("RR", std::bind(&CommandManager::do_restart, this, _1, _2, _3, _4));
    command_map.emplace("TEST", std::bind(&CommandManager::do_give_status, this, _1, _2, _3, _4));
    command_map.emplace("SIM", std::bind(&CommandManager::do_sim, this, _1, _2, _3, _4));
    command_map.emplace("SIMP", std::bind(&CommandManager::do_simp, this, _1, _2, _3, _4));
    command_map.emplace("CAL", std::bind(&CommandManager::do_cal, this, _1, _2, _3, _4));
    command_map.emplace("MEC", std::bind(&CommandManager::do_mec, this, _1, _2, _3, _4));
    command_map.emplace("GTLOGS", std::bind(&CommandManager::do_logs, this, _1, _2, _3, _4));
}

int CommandManager::processCommand(const char *cmd_buff, SerialManager &ser, MissionManager &info, SensorManager &sensors)
{
    // Extract fields from command buffer

    struct command_packet
    {
        char *keyword = nullptr;
        char *team_id = nullptr;
        char *command = nullptr;
        char *data = nullptr;
    };

    command_packet packet;

    char cmd_buff_copy[CMD_BUFF_SIZE];
    strncpy(cmd_buff_copy, cmd_buff, CMD_BUFF_SIZE);
    cmd_buff_copy[CMD_BUFF_SIZE - 1] = '\0';

    char *token = strtok(cmd_buff_copy, ",");
    int token_cnt = 0;

    while(token != NULL)
    {
        switch(token_cnt)
        {
            case 0:
                packet.keyword = token;
                break;
            case 1:
                packet.team_id = token;
                break;
            case 2:
                packet.command = token;
                break;
        }
        token_cnt++;

        if(token_cnt == 3)
        {
          token = strtok(NULL, "");
          packet.data = token;
        }
        else
        {
          token = strtok(NULL, ",");
        }
    }

    // Check validity
    if(!packet.keyword || !packet.team_id || !packet.command || !packet.data)
    {
        ser.sendErrorMsg("COMMAND REJECTED: FORMAT IS INCORRECT.");
        ser.sendErrorDataMsg("RECEIVED: %s\n", cmd_buff);
        return 0;
    }

    if(strcmp(packet.keyword, "CMD") != 0)
    {
        ser.sendErrorMsg("COMMAND REJECTED: FIRST FIELD MUST BE 'CMD'.");
        return 0;
    }

    if(atoi(packet.team_id) != TEAM_ID)
    {
        ser.sendErrorDataMsg("COMMAND REJECTED: TEAM ID DOES NOT MATCH EXPECTED VALUE OF %d", TEAM_ID);
        return 0;
    }

    // Check if the command is in the map and call the corresponding function
    auto iter = command_map.find(packet.command);
    if (iter == command_map.end())
    {
        ser.sendErrorMsg("COMMAND REJECTED: NOT A COMMAND");
        return 0;
    }
    iter->second(ser, info, sensors, packet.data);
    return 1;
}

// Toggle mission telemetry
void CommandManager::do_cx(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
    if(strcmp(data, "ON") == 0)
    {
        if(info.getOpState() == IDLE && (info.isAltCalibrated() == 1 || info.getOpMode() == OPMODE_SIM))
        {
            info.clearPacketCount();
            ser.sendInfoMsg("STARTING TELEMETRY TRANSMISSION.");
            info.setOpState(LAUNCH_PAD);
            info.beginPref("xb-set", false);
            int state_int = static_cast<int>(LAUNCH_PAD);
            info.putPrefInt("opstate", state_int);
            info.endPref();
        }
        else if(info.getOpState() != IDLE)
        {
            ser.sendErrorMsg("TRANSMISSION IS ALREADY ON.");
        }
        else
        {
            ser.sendErrorMsg("CANNOT START TELEMETRY BEFORE CALIBRATING ALTITUDE!");
        }
    }
    else if(strcmp(data, "OFF") == 0)
    {
        if(info.getOpState() != IDLE)
        {
            info.setOpState(IDLE);
            info.beginPref("xb-set", false);
            int state_int = static_cast<int>(IDLE);
            info.putPrefInt("opstate", state_int);
            info.endPref();
            info.setAltCalOff();
            ser.sendInfoDataMsg("ENDING PAYLOAD TRANSMISSION.{%s|%s}",
                sensors.op_mode_to_string(info.getOpMode(), 1), sensors.op_state_to_string(info.getOpState()));
        }
        else
        {
            ser.sendErrorMsg("TRANSMISSION IS ALREADY OFF!");
        }
    }
    else
    {
        ser.sendErrorMsg("DATA IS NOT VALID; SEND ON/OFF");
    }
}

void CommandManager::do_st(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
    int h,m,s;
    if(strcmp(data, "GPS") == 0)
    {
        char time_str[DATA_SIZE];
        sensors.getGpsTime(time_str);
        if(sscanf(time_str, "%d:%d:%d", &h, &m, &s) == 3)
        {
          sensors.setRtcTime(s,m,h);
          sensors.getRtcTime(time_str);
          ser.sendInfoDataMsg("Set RTC time to %s", time_str);
        }
        else
        {
          ser.sendErrorMsg("Could not get GPS time in format HH:MM:SS!");
        }
    }
    else if(sscanf(data, "%d:%d:%d", &h, &m, &s) == 3)
    {
        sensors.setRtcTime(s,m,h);
        char time_str[DATA_SIZE];
        sensors.getRtcTime(time_str);
        ser.sendInfoDataMsg("Set RTC time to %s", time_str);
    }
    else
    {
        ser.sendErrorMsg("DATA IS NOT VALID; SEND 'GPS' OR UTC TIME");
    }
}

void CommandManager::do_give_status(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  ser.sendInfoDataMsg("CANSAT IS ONLINE.{%s|%s}",
      sensors.op_mode_to_string(info.getOpMode(), 1), sensors.op_state_to_string(info.getOpState()));
} // END: do_give_status

void CommandManager::do_restart(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  ser.sendInfoMsg("Attempting to restart processor!");
  ESP.restart();
} // END: do_restart

void CommandManager::do_sim(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  if(info.getOpState() != IDLE)
  {
    ser.sendErrorMsg("SIMULATION MODE CANNOT BE CHANGED WHILE TRANSMISSION IS ON");
    return;
  }

  if(strcmp(data, "ENABLE") == 0)
  {
    switch(info.getSimStatus())
    {
      case SIM_OFF:
        {
          info.setSimStatus(SIM_EN);
          info.beginPref("xb-set", false);
          int sim_status_int = static_cast<int>(info.getSimStatus());
          info.putPrefInt("simst", sim_status_int);
          info.endPref();
          ser.sendInfoMsg("SIMULATION MODE ENABLED");
          break;
        }
      case SIM_EN:
        {
          ser.sendErrorMsg("SIMULATION MODE IS ALREADY ENABLED");
          break;
        }
      case SIM_ON:
        {
          ser.sendErrorMsg("SIMULATION MODE IS ALREADY ACTIVATED");
          break;
        }
    }
  }
  else if(strcmp(data, "ACTIVATE") == 0)
  {
    switch(info.getSimStatus())
    {
      case SIM_EN:
        {
          info.setSimStatus(SIM_ON);
          info.setOpMode(OPMODE_SIM);
          info.waitingForSimp();
          info.beginPref("xb-set", false);
          int sim_status_int = static_cast<int>(info.getSimStatus());
          info.putPrefInt("simst", sim_status_int);
          int op_mode_int = static_cast<int>(info.getOpMode());
          info.putPrefInt("opmode", op_mode_int);
          info.endPref();
          ser.sendInfoDataMsg("SIMULATION MODE IS ACTIVE{%s|%s}",
            sensors.op_mode_to_string(info.getOpMode(), 1), sensors.op_state_to_string(info.getOpState()));
          break;
        }
      case SIM_OFF:
        {
          ser.sendErrorMsg("SIMULATION MODE IS NOT ENABLED");
          break;
        }
      case SIM_ON:
        {
          ser.sendErrorMsg("SIMULATION MODE IS ALREADY ACTIVATED");
          break;
        }
    }
  }
  else if(strcmp(data, "DISABLE") == 0)
  {
    switch(info.getSimStatus())
    {
      case SIM_ON:
      case SIM_EN:
        {
          info.setSimStatus(SIM_OFF);
          info.setOpMode(OPMODE_FLIGHT);
          info.beginPref("xb-set", false);
          int sim_status_int = static_cast<int>(info.getSimStatus());
          info.putPrefInt("simst", sim_status_int);
          int op_mode_int = static_cast<int>(info.getOpMode());
          info.putPrefInt("opmode", op_mode_int);
          info.endPref();
          ser.sendInfoDataMsg("SET CANSAT TO FLIGHT MODE.{%s|%s}",
            sensors.op_mode_to_string(info.getOpMode(), 1), sensors.op_state_to_string(info.getOpState()));
          break;
        }
      case SIM_OFF:
        {
          ser.sendErrorMsg("SIMULATION MODE ALREADY DISABLED.");
          break;
        }
    }
  }
  else
  {
    ser.sendErrorDataMsg("UNRECOGNIZED SIM COMMAND: '%s'", data);
  }
} // END: do_sim()

void CommandManager::do_simp(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  // Check if we are in simulation mode
  if(info.getOpMode() == OPMODE_SIM)
  {
    int pressure;
    if(sscanf(data, "%d", &pressure) == 1)
    {
      float alt = sensors.pressure_to_alt(pressure/100.0);
      if(info.isWaitingSimp())
      {
          info.setAltCalibration(alt);
          info.simpRecv();
      }
      sensors.setAltData(alt-info.getLaunchAlt());
      info.setSimpData(pressure);
    }
    else
    {
      ser.sendErrorDataMsg("ERROR: SIMP DATA FORMAT IS INCORRECT: %s", data);
    }
  }
  else
  {
    ser.sendErrorMsg("CANNOT RECEIVE SIMP CMD, CANSAT IS IN FLIGHT MODE");
  }
} // END: do_simp()

void CommandManager::do_cal(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  info.setAltCalibration(sensors.pressure_to_alt(sensors.getPressure()*10));
  info.beginPref("xb-set", false);
  info.putPrefFloat("grndalt", info.getLaunchAlt());
  info.endPref();
  ser.sendInfoDataMsg("Launch Altitude calibrated to %f", info.getLaunchAlt());
} // END: do_Cal()

void CommandManager::do_mec(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  char data_copy[CMD_BUFF_SIZE];
  strcpy(data_copy, data);

  const char *delim = ":";
  char *token;

  char mec[16];
  char val[16];

  token = strtok(data_copy, delim);
  if(token == NULL)
  {
    ser.sendErrorMsg("MEC FORMAT IS INCORRECT, DATA SHOULD CONTAIN 'MEC:VAL'");
    return;
  }
  else
  {
    strcpy(mec, token);
  }

  token = strtok(NULL, delim);
  if(token == NULL)
  {
    ser.sendErrorMsg("MEC FORMAT IS INCORRECT, DATA SHOULD CONTAIN 'MEC:VAL'");
    return;
  }
  else
  {
    strcpy(val, token);
  }

  if(strcmp(mec, "TEST") == 0)
  {
    ser.sendInfoMsg("RECEIVED TEST MEC COMMAND!");
  }
  else
  {
    ser.sendErrorDataMsg("ERROR: UNRECOGNIZED MEC COMMAND: %s", mec);
  }
}

void CommandManager::do_logs(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  // Only do this if IDLE
  if(info.getOpState() != IDLE)
  {
    ser.sendErrorMsg("CANNOT SEND LOG DATA WHILE CANSAT IS NOT IDLE!");
    return;
  }

  // TODO: Add once EEPROM is set up
  ser.sendInfoMsg("GOT TEST LOG COMMAND!");

  log.close();
}
