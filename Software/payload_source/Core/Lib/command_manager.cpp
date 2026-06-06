/**
  ******************************************************************************
  * @file           : command_manager.cpp
  * @author         : RSX 2025-2026
  * @brief          : Processes commands received by ground station
  ******************************************************************************
  */

#include "command_manager.hpp"

CommandManager::CommandManager()
{
    // Initialize function map
    using namespace std::placeholders;
    command_map.emplace("CX", std::bind(&CommandManager::do_cx, this, _1, _2, _3, _4));
    command_map.emplace("ST", std::bind(&CommandManager::do_st, this, _1, _2, _3, _4));
    command_map.emplace("RST", std::bind(&CommandManager::do_restart, this, _1, _2, _3, _4));
    command_map.emplace("TEST", std::bind(&CommandManager::do_give_status, this, _1, _2, _3, _4));
    command_map.emplace("SIM", std::bind(&CommandManager::do_sim, this, _1, _2, _3, _4));
    command_map.emplace("SIMP", std::bind(&CommandManager::do_simp, this, _1, _2, _3, _4));
    command_map.emplace("CAL", std::bind(&CommandManager::do_cal, this, _1, _2, _3, _4));
    command_map.emplace("MEC", std::bind(&CommandManager::do_mec, this, _1, _2, _3, _4));
    command_map.emplace("LOG", std::bind(&CommandManager::do_logs, this, _1, _2, _3, _4));
    command_map.emplace("CAL2", std::bind(&CommandManager::do_cal2, this, _1, _2, _3, _4));
    command_map.emplace("CTRL", std::bind(&CommandManager::do_ctrl, this, _1, _2, _3, _4));
    command_map.emplace("GPS", std::bind(&CommandManager::do_gps, this, _1, _2, _3, _4));
}

uint8_t CommandManager::processCommand(const char *cmd_buff, SerialManager &ser, MissionManager &info, SensorManager &sensors)
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
    if(!packet.keyword || !packet.team_id || !packet.command)
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
    if(iter == command_map.end())
    {
        ser.sendErrorMsg("COMMAND REJECTED: NOT A COMMAND");
        return 0;
    }
    iter->second(ser, info, sensors, packet.data);
    return 1;
}

// Change flight control mode
void CommandManager::do_ctrl(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
	if(!data)
	{
		ser.sendErrorMsg("COMMAND 'CTRL' REJECTED: DID NOT RECEIVE ANY MODE");
		return;
	}
	if(strcmp(data, "AUTO") == 0)
	{
		info.setFlightCtrl(FlightCtrl::AUTONOMOUS);
	}
	else if(strcmp(data, "MANUAL") == 0)
	{
		info.setFlightCtrl(FlightCtrl::MANUAL);
	}
	else
	{
		ser.sendErrorDataMsg("DATA IS NOT VALID: RECEIVED %s INSTEDA OF AUTO/MANUAL", data);
	}
}

// Toggle mission telemetry
void CommandManager::do_cx(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
    if(!data)
    {
        ser.sendErrorMsg("COMMAND 'CX' REJECTED: DID NOT RECEIVE ON/OFF");
        return;
    }

    if(strcmp(data, "ON") == 0)
    {
        if(info.getOpState() == IDLE && (info.isAltCalibrated() == true || info.getOpMode() == OPMODE_SIM))
        {
            sensors.EEPROM_resetData();
            info.init_params();
            ser.sendInfoMsg("ATTEMPTING TO START MISSION.");
            info.setOpState(LAUNCH_PAD);
            sensors.EEPROM_updateState(LAUNCH_PAD);
            status_update(ser, info);
            sensors.EEPROM_resetLog();
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
            sensors.EEPROM_updateState(IDLE);
            info.reset_params();
            ser.sendInfoDataMsg("ENDING PAYLOAD TRANSMISSION.{%s|%s|%s}",
              op_mode_to_string(info.getOpMode(), 1), op_state_to_string(info.getOpState()), flight_ctrl_to_string(info.getFlightCtrl()));
        }
        else
        {
            ser.sendErrorMsg("TRANSMISSION IS ALREADY OFF!");
        }
    }
    else
    {
        ser.sendErrorDataMsg("DATA IS NOT VALID: RECEIVED %s INSTEDA OF ON/OFF", data);
    }
}

void CommandManager::do_st(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
    if(!data)
    {
        ser.sendErrorMsg("COMMAND 'ST' REJECTED: DID NOT TIME DATA");
        return;
    }

    int h,m,s;
    if(strcmp(data, "GPS") == 0)
    {
        char time_str[DATA_SIZE];
        char rtc_time_str[DATA_SIZE];
        sensors.updateGPS(ser);
        sensors.getGPSTime(time_str);
        if(sscanf(time_str, "%d:%d:%d", &h, &m, &s) == 3)
        {
          sensors.setRTCTime(h, m, s);
          sensors.getRTCTime(rtc_time_str);
          ser.sendInfoDataMsg("Got GPS time %s, RTC set to %s", time_str, rtc_time_str);
        }
        else
        {
          ser.sendErrorMsg("Could not get GPS time in format HH:MM:SS!");
        }
    }
    else if(sscanf(data, "%d:%d:%d", &h, &m, &s) == 3)
    {
        sensors.setRTCTime(h, m, s);
        char time_str[DATA_SIZE];
        sensors.getRTCTime(time_str);
        ser.sendInfoDataMsg("Attempted to set RTC time to %s, got %s", data, time_str);
    }
    else
    {
        ser.sendErrorMsg("DATA IS NOT VALID; SEND 'GPS' OR UTC TIME");
    }
}

void CommandManager::do_give_status(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  ser.sendInfoDataMsg("CANSAT IS ONLINE");
  status_update(ser, info);
} // END: do_give_status

void CommandManager::status_update(SerialManager &ser, MissionManager &info)
{
	ser.sendInfoDataMsg("{%s|%s|%s|%.1f}",
			op_mode_to_string(info.getOpMode(), 1), op_state_to_string(info.getOpState()), flight_ctrl_to_string(info.getFlightCtrl()),
			info.getLaunchAlt());
}

void CommandManager::do_restart(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  ser.sendInfoMsg("Attempting to restart processor!");
  HAL_NVIC_SystemReset();
} // END: do_restart

void CommandManager::do_sim(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  if(!data)
  {
      ser.sendErrorMsg("COMMAND 'SIM' REJECTED: DID NOT RECEIVE CONTROL DATA");
      return;
  }

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
          sensors.EEPROM_updateMode(info.getOpMode());
          ser.sendInfoDataMsg("SIMULATION MODE IS ACTIVE");
          status_update(ser, info);
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
	if(info.getOpMode() == OPMODE_SIM)
	{
		info.setSimStatus(SIM_OFF);
		info.setOpMode(OPMODE_FLIGHT);
		sensors.EEPROM_updateMode(info.getOpMode());
		ser.sendInfoDataMsg("SET CANSAT TO FLIGHT MODE.");
		status_update(ser, info);
	}

    switch(info.getSimStatus())
    {
      case SIM_ON:
      case SIM_EN:
        {
          info.setSimStatus(SIM_OFF);
          info.setOpMode(OPMODE_FLIGHT);
          sensors.EEPROM_updateMode(info.getOpMode());
          ser.sendInfoDataMsg("SET CANSAT TO FLIGHT MODE.");
          status_update(ser, info);
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
  if(!data)
  {
      ser.sendErrorMsg("COMMAND 'SIMP' REJECTED: DID NOT RECEIVE DATA");
      return;
  }

  // Check if we are in simulation mode
  if(info.getOpMode() == OPMODE_SIM)
  {
    int pressure;
    if(sscanf(data, "%d", &pressure) == 1)
    {
      float alt = pressure_to_alt(pressure);
      if(info.isWaitingSimp())
      {
          info.setAltCalibration(alt);
          info.simpRecv();
      }
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
  int ret = sensors.updateBMP();
  if(ret != 0)
  {
	  ser.sendErrorDataMsg("BMP ERROR %d", ret);
	  return;
  }
  info.setAltCalibration(pressure_to_alt(sensors.getPressure()));
  sensors.EEPROM_updateAltitude(info.getLaunchAlt());
  ser.sendInfoDataMsg("Launch Altitude calibrated to %.1f m", info.getLaunchAlt());
  status_update(ser, info);
} // END: do_Cal()

void CommandManager::do_cal2(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  if(!data)
  {
    ser.sendErrorMsg("COMMAND 'CAL2' REJECTED: DID NOT RECEIVE DATA");
    return;
  }

  if(strcmp(data, "MAG") == 0)
  {

  }
  else if(strcmp(data, "EGG") == 0)
  {
	  sensors.updateBMP();
	  info.setEggAlt(pressure_to_alt(sensors.getPressure()));
	  ser.sendInfoDataMsg("Set egg altitude to %f", info.getEggAlt());
  }
  else if(strcmp(data, "EGG2") == 0)
  {
	  info.setEggAlt(0.0);
	  ser.sendInfoDataMsg("Erased egg altitude to %f", info.getEggAlt());
  }
  else if (strcmp(data, "BNO") == 0)
  {
    ser.sendInfoMsg("Starting auto calibration...");
    sensors.BNO_calibrate(ser);
    int gyro_acc = 0;
    int accel_acc = 0;
    int mag_acc = 0;
    float * rawRotVec = new float[5];
    float * rawAccel = new float[4];
    float * rawMag = new float[4];
    int i = 0;
    uint32_t timeout = HAL_GetTick();
    while ((gyro_acc < 3 || accel_acc < 3 || mag_acc < 3) && HAL_GetTick() - timeout < 30000)
    {
      sensors.updateBNO();
      sensors.getGameRotationVector(rawRotVec);
      sensors.getRawAccel(rawAccel);
      sensors.getRawMag(rawMag);
      gyro_acc = (int)rawRotVec[4];
      accel_acc = (int)rawAccel[3];
      mag_acc = (int)rawMag[3];
      if (i % 10 == 0) {
        ser.sendInfoDataMsg("Current calibration accuracy - Gyro: %d, Accel: %d, Mag: %d", gyro_acc, accel_acc, mag_acc);
      }
      i++;
      HAL_Delay(50);
    }
    ser.sendInfoMsg("Calibration complete!");
    sensors.BNO_saveCalibration(ser);
    sensors.BNO_disableCalibration(ser);
    delete[] rawRotVec;
    delete[] rawAccel;
    delete[] rawMag;
  }
  else
  {
    ser.sendErrorDataMsg("ERROR: UNRECOGNIZED CAL COMMAND: %s", data);
  }
} // END: do_Cal2()

void CommandManager::do_mec(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
  if(!data)
  {
      ser.sendErrorMsg("COMMAND 'MEC' REJECTED: DID NOT RECEIVE ANY DATA");
      return;
  }

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

  if(strcmp(mec, "REL") == 0)
  {
	  int val_int = atoi(val);

	  if(val_int == 0)
	  {
		  sensors.activate_nosecone_release();
		  if(info.getOpState() != DESCENT && info.getOpState() != PROBE_RELEASE && info.getOpState() != PAYLOAD_RELEASE)
		  {
			  info.setOpState(DESCENT);
			  sensors.EEPROM_updateState(DESCENT);
			  info.nosecone_rel();
		  }
	  }
	  else if(val_int == 1)
	  {
		  sensors.activate_probe_release();
		  if(info.getOpState() != PROBE_RELEASE && info.getOpState() != PAYLOAD_RELEASE)
		  {
			  info.setOpState(PROBE_RELEASE);
			  sensors.EEPROM_updateState(PROBE_RELEASE);
			  info.probe_rel();
		  }
	  }
	  else if(val_int == 2)
	  {
		  sensors.activate_wing_deployment();
		  if(info.getOpState() != PROBE_RELEASE && info.getOpState() != PAYLOAD_RELEASE)
		  {
			  info.setOpState(PROBE_RELEASE);
			  sensors.EEPROM_updateState(PROBE_RELEASE);
			  info.probe_rel();
			  info.wing_rel();
		  }
	  }
	  else if(val_int == 3)
	  {
		  sensors.activate_egg_release();
		  if(info.getOpState() != PAYLOAD_RELEASE)
		  {
			  info.setOpState(PAYLOAD_RELEASE);
			  sensors.EEPROM_updateState(PAYLOAD_RELEASE);
			  info.egg_rel();
		  }
	  }
	  else
	  {
		  ser.sendErrorMsg("ERROR: RELEASE COMMAND FORMAT INCORRECT");
	  }
  }
  else if(strcmp(mec, "SERVO") == 0)
  {
	  const char *sep = strchr(val, '|');
	  if(sep != NULL)
	  {
		  char left[8], right[8];
		  size_t len_left = sep - val;

		  strncpy(left, val, len_left);
		  left[len_left] = '\0';

		  strcpy(right, sep + 1);

		  int servo_num = atoi(left);
		  float servo_val = atoi(right);

		  switch(servo_num)
		  {
		  	  case 0:
		  		  sensors.writeNoseconeServo(servo_val);
		  		  ser.sendInfoDataMsg("Wrote %.1f to nosecone servo.", servo_val);
		  		  break;
		  	  case 1:
		  		  sensors.writeContainerServo(servo_val);
		  		  ser.sendInfoDataMsg("Wrote %.1f to container servo.", servo_val);
		  		  break;
		  	  case 2:
		  		  sensors.writeElevatorServo(servo_val);
		  		  ser.sendInfoDataMsg("Wrote %.1f to elevator servo.", servo_val);
		  		  break;
		  	  case 3:
		  		  sensors.writeAileronServo(servo_val);
		  		  ser.sendInfoDataMsg("Wrote %.1f to aileron servo.", servo_val);
		  		  break;
		  	  case 4:
		  		  sensors.writeEggServo(servo_val);
		  		  ser.sendInfoDataMsg("Wrote %.1f to egg servo.", servo_val);
		  		  break;
		  	  default:
		  		  ser.sendErrorMsg("ERROR: SERVO ID DOES NOT MATCH 0-6");
		  		  break;
		  }
	  }
	  else
	  {
		  ser.sendErrorMsg("ERROR: SERVO COMMAND FORMAT INCORRECT, DID NOT RECEIVE '#|VAL'");
	  }
  }
  else if(strcmp(mec, "DC") == 0)
  {
	  int DC_val = atoi(val);
	  if(DC_val == 0)
	  {
		  sensors.stopMotor();
		  ser.sendInfoMsg("Motor stopped.");
	  }
	  else
	  {
		  uint8_t  direction = (DC_val > 0) ? 1 : 0;
		  uint32_t duration  = (uint32_t)(DC_val > 0 ? DC_val : -DC_val);
		  sensors.writeMotor(direction, duration);
		  ser.sendInfoDataMsg("Motor running %s for %lu ms.", direction ? "forward" : "reverse", (unsigned long)duration);
	  }
  }
  else if(strcmp(mec, "CAM1") == 0)
  {
	  if(info.getOpState() != IDLE && info.getOpState() != LANDED)
	  {
		  ser.sendErrorMsg("ERROR: CANNOT CHANGE CAMERA IN CURRENT STATE!");
		  return;
	  }
	  int cam_id = atoi(val);
	  if(cam_id == 0)
	  {
		  // Ground
		  sensors.ground_runcam_start();
		  ser.sendInfoMsg("Attempted to start ground cam");
	  }
	  else if(cam_id == 1)
	  {
		  // Payload
		  sensors.payload_runcam_start();
		  ser.sendInfoMsg("Attempted to start payload cam");
	  }
	  else
	  {
		  ser.sendErrorMsg("ERROR: DID NOT RECEIVE VALID CAMERA ID!");
	  }
  }
  else if(strcmp(mec, "CAM0") == 0)
  {
	  if(info.getOpState() != IDLE || info.getOpState() != LANDED)
	  {
		  ser.sendErrorMsg("ERROR: CANNOT CHANGE CAMERA IN CURRENT STATE!");
		  return;
	  }
	  int cam_id = atoi(val);
	  if(cam_id == 0)
	  {
		  // Ground
		  sensors.ground_runcam_stop();
		  ser.sendInfoMsg("Attempted to stop ground cam");
	  }
	  else if(cam_id == 1)
	  {
		  // Payload
		  sensors.payload_runcam_stop();
		  ser.sendInfoMsg("Attempted to stop payload cam");
	  }
	  else
	  {
		  ser.sendErrorMsg("ERROR: DID NOT RECEIVE VALID CAMERA ID!");
	  }
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

  sensors.EEPROM_replayLog(100, ser);
}

void CommandManager::do_gps(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data)
{
	if(!data)
	{
	  ser.sendErrorMsg("COMMAND 'GPS' REJECTED: DID NOT RECEIVE ANY DATA");
	  return;
	}

	char buf[DATA_SIZE];
	strncpy(buf, data, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	float vals[3];
	int val_count = 0;

	for(char *tok = strtok(buf, "|"); tok != nullptr; tok = strtok(nullptr, "|"))
	{
		if(val_count >= 3)
		{
			val_count++;
			break;
		}

		char *end = nullptr;
		float val = strtof(tok, &end);

		if(end == tok || *end != '\0')
		{
			ser.sendErrorMsg("COMMAND 'GPS' REJECTED: RECEIVED INVALID FIELD");
			return;
		}
		vals[val_count++] = val;
	}

	if(val_count < 2)
	{
		ser.sendErrorDataMsg("COMMAND 'GPS' REJECTED: RECEIVED %d FIELDS", val_count);
		return;
	}

	info.set_landing_coords(vals[0], vals[1]);
	ser.sendInfoDataMsg("Set landing coords to %.4f %.4f", info.get_landing_lat(), info.get_landing_lon());
}
