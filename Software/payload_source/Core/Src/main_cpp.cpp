#include "glider_ekf.h"
#include "drv.hpp"
#include "main.h"
#include "global_includes.hpp"
#include "mission_manager.hpp"
#include "sensor_manager.hpp"
#include "serial_manager.hpp"
#include "telemetry_manager.hpp"
#include "command_manager.hpp"
#include "PathGuidance.hpp"
#include "controller.hpp"

// ── Competition field GPS boundary corners ────────────────────────────────────
// Four corners of the allowed flight area in decimal degrees (lat, lon).
// At planning time each corner is converted to the EKF local NED frame;
// the axis-aligned bounding box of those NED points becomes the hard flight boundary.
// Midpoint / EKF home: 38.375977, -79.607846
static constexpr float kFieldCorners[4][2] = {
    {38.381305f, -79.607000f},  // corner NW
    {38.377439f, -79.602209f},  // corner NE
    {38.374956f, -79.613029f},  // corner W
    {38.372403f, -79.606825f},  // corner S
};

// ── Landing axis ──────────────────────────────────────────────────────────────
// Runway line equation (WGS-84, decimal degrees):  lon = -1.5561798 * lat - 19.8879258
// Axis bearing derivation:
//   direction vector in lat/lon space: (dLat=1, dLon=-1.5561798)
//   in NED (m): dN = dLat*111320, dE = dLon*111320*cos(38.376°)
//             = (111320,  -1.5561798 * 87268)
//             = (111320,  -135805)
//   bearing = atan2(dE, dN) = atan2(-135805, 111320) ≈ -0.884 rad
//   ≈ -50.6° (NW–SE axis); the planner picks the better approach end automatically.
static constexpr float kLandingAxisRad = -0.884f;

// ── Glider aerodynamic and geometric parameters ───────────────────────────────
// Tune these to match the actual airframe.
static constexpr float kGlideRatio      = 3.0f;  // nominal glide ratio (horizontal/vertical)
static constexpr float kMinTurnRadius   = 25.f;  // minimum turn radius (m) — physical limit
static constexpr float kMaxSpiralRadius = 70.f;  // maximum loiter spiral radius (m)
static constexpr float kApproachLength  = 40.f;  // final straight approach leg length (m)
static constexpr float kLookaheadDrop   = 8.f;   // carrot lookahead in altitude drop (m)

// Forward azimuth (NED bearing) from point 1 to point 2, both in decimal degrees.
// Returns radians: 0 = North, +π/2 = East.
[[maybe_unused]]
static float gps_bearing_rad(float lat1_deg, float lon1_deg, float lat2_deg, float lon2_deg)
{
    static constexpr float kPi = 3.14159265358979f;
    const float lat1 = lat1_deg * kPi / 180.f;
    const float lat2 = lat2_deg * kPi / 180.f;
    const float dLon = (lon2_deg - lon1_deg) * kPi / 180.f;
    const float y = sinf(dLon) * cosf(lat2);
    const float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dLon);
    return atan2f(y, x);  // range [-π, +π]
}

extern "C" volatile uint8_t send_flag;
extern "C" volatile uint8_t pvd_flag;
extern "C" volatile uint8_t update_flag;
extern "C" volatile uint8_t bno_flag;
extern "C" UART_HandleTypeDef huart1;
extern "C" TIM_HandleTypeDef htim1;
extern "C" TIM_HandleTypeDef htim2;
extern "C" TIM_HandleTypeDef htim3;
extern "C" TIM_HandleTypeDef htim4;
extern "C" TIM_HandleTypeDef htim8;
extern "C" SPI_HandleTypeDef hspi1;
extern "C" I2C_HandleTypeDef hi2c1;
extern "C" volatile char rx_buff[128];
extern "C" volatile uint8_t cmd_ready;

uint32_t nosecone_rel__payload_rel_timer = 0;
uint32_t wing_servo_timer = 0;
uint32_t egg_timer = 0;
uint32_t bno_update_timer = 0;
OperatingState update_state(SensorManager &sensors, MissionManager &mgr, OperatingState current_state);

extern "C" void main_cpp()
{

    SerialManager serial(huart1);

    MissionManager mission_mgr;
    CommandManager cmd_mgr;
    TelemetryManager telemetry_mgr;

    SensorManager sensors;

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)){
		serial.sendErrorMsg("Reset Reason: low power reset");
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)){
		serial.sendErrorMsg("Reset Reason: window watchdog reset");
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)){
		serial.sendErrorMsg("Reset Reason: independent watchdog reset");
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)){
		serial.sendErrorMsg("Reset Reason: software reset");
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)){
		serial.sendErrorMsg("Reset Reason: external pin reset");
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)){
		serial.sendErrorMsg("Reset Reason: brown-out reset");
	}
	else {
		serial.sendErrorMsg("Reset Reason: unknown");
	}

	__HAL_RCC_CLEAR_RESET_FLAGS();

	sensors.startSensors(serial, &hi2c1, &hspi1, SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, &htim2, &htim3, &htim4);

	glider_ekf_init();

	rsx::PathGuidance guidance;

	FlightControllers controller(RollControllerConfig{}, PitchControllerConfig{}, ServoHardwareConfig{});
	uint32_t ctrl_timer = 0;
	bool plan_done = false;

    struct recovery_data recovery = sensors.EEPROM_getRecoveryData();

    mission_mgr.setOpState(recovery.state);
	mission_mgr.setFlightCtrl(AUTONOMOUS); // always start in autonomous mode

    if(recovery.state != IDLE)
    {
        serial.sendErrorMsg("Performing recovery as processor was not in IDLE state! Telemetry should resume!");
		serial.sendErrorMsg("CPL forced into AUTONOMOUS FLIGHT CTRL for recovery.");
        // Get packet count, launch altitude
        mission_mgr.setAltCalibration(recovery.launch_altitude);
        mission_mgr.setOpMode(recovery.mode);
        mission_mgr.setPacketCount(recovery.packet_count);
        mission_mgr.update_max_alt(recovery.max_alt);
        if(recovery.nosecone_flag)
        {
        	mission_mgr.nosecone_rel();
        }
        if(recovery.egg_flag)
        {
        	mission_mgr.egg_rel();
        }
        if(recovery.probe_flag)
        {
        	mission_mgr.probe_rel();
        }
        if(recovery.wing_flag)
        {
        	mission_mgr.wing_rel();
        }

        // Reset the timers
        nosecone_rel__payload_rel_timer = HAL_GetTick();
        wing_servo_timer = HAL_GetTick();
        egg_timer = HAL_GetTick();
    }
    else
    {
    	serial.sendInfoMsg("Setup completed, entering IDLE mode");
    }

    char cmd_buff[CMD_BUFF_SIZE];
    char send_buff[DATA_BUFF_SIZE];

    HAL_TIM_Base_Start_IT(&htim2);

    while(1)
    {
        while(mission_mgr.getOpState() == IDLE)
        {
        	if(bno_flag)
        	{
        		bno_flag = 0;
        		sensors.updateGPS(serial);
				if(sensors.BNO_dataReady())
				{
					sensors.updateBNO();
				}
        	}

            if(cmd_ready)
            {
            	memcpy(cmd_buff, (const char*)rx_buff, CMD_BUFF_SIZE);
            	cmd_ready = 0;

                if(cmd_mgr.processCommand(cmd_buff, serial, mission_mgr, sensors))
                {
                    mission_mgr.setLastCommand(cmd_buff);
                }

            }
            sensors.updateMotor();
            HAL_Delay(10);
        }

        if(mission_mgr.getOpMode() == OPMODE_SIM)
        {
            serial.sendInfoMsg("SIM_START");

            // Wait until first simulation packet is received
            while(mission_mgr.getOpMode() == OPMODE_SIM && mission_mgr.isWaitingSimp())
            {
            	if(cmd_ready)
				{
					memcpy(cmd_buff, (const char*)rx_buff, CMD_BUFF_SIZE);
					cmd_ready = 0;

					if(cmd_mgr.processCommand(cmd_buff, serial, mission_mgr, sensors))
					{
						mission_mgr.setLastCommand(cmd_buff);
					}

				}
				HAL_Delay(10);

				if(bno_flag)
				{
					bno_flag = 0;
					sensors.updateGPS(serial);
					if(sensors.BNO_dataReady())
					{
						sensors.updateBNO();
					}
				}
            }
        }

        serial.sendInfoMsg("MISSION STARTING!");

        HAL_TIM_Base_Start_IT(&htim1);
        HAL_TIM_Base_Start_IT(&htim8);

        bno_update_timer = HAL_GetTick();
        ctrl_timer = HAL_GetTick();

        // GPS-derived position & velocity state (reset each mission run)
        double   gps_prev_lat  = 0.0, gps_prev_lon = 0.0;
        float    gps_prev_alt  = 0.f;
        uint32_t gps_prev_tick = 0;
        bool     gps_has_prev  = false;
        float    gps_vn = 0.f, gps_ve = 0.f, gps_vd = 0.f;  // NED m/s

        while(mission_mgr.getOpState() != IDLE)
        {
			sensors.updateMotor();
			
            if(cmd_ready)
            {
            	memcpy(cmd_buff, (const char*)rx_buff, CMD_BUFF_SIZE);
            	cmd_ready = 0;

                if(cmd_mgr.processCommand(cmd_buff, serial, mission_mgr, sensors))
                {
                    mission_mgr.setLastCommand(cmd_buff);
                }
            }

            if(send_flag)
            {
            	/*
				float pos[3];
				float vel[3];
				float x_q[4];
				ekf_get_pos(pos);
				ekf_get_vel(vel);
				ekf_get_quaternion(x_q);
				serial.sendInfoDataMsg("EKF State: NED Position (%.1f, %.1f, %.1f), Velocity (%.1f, %.1f, %.1f)", pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]);
				float raw_accel[4];
				sensors.getLinearAccel(raw_accel);
				serial.sendInfoDataMsg("Linear Accel: (%.3f, %.3f, %.3f) m/s^2, Accuracy: %d", raw_accel[0], raw_accel[1], raw_accel[2], (int)raw_accel[3]);
				float rpy[3];
				quat_to_rpy(x_q, rpy);
				serial.sendInfoDataMsg("EKF State: RPY (%.3f, %.3f, %.3f)", rpy[0], rpy[1], rpy[2]);
				*/
				telemetry_mgr.sampleSensors(sensors, mission_mgr, serial);
            	telemetry_mgr.build_data_str(send_buff, sizeof(send_buff));

            	serial.sendTelemetry(send_buff);

				if(mission_mgr.logfile_ok() && !sensors.EEPROM_addLogLine(send_buff))
				{
					serial.sendErrorMsg("Warning: Unable to add line to logfile, disabling logfile writes!");
					mission_mgr.disableLogfile();
				}

            	send_flag = 0;

            	if(mission_mgr.getOpState() == APOGEE)
            	{
            		mission_mgr.apogee_packet_sent();
            	}
            }

            if(update_flag)
            {
            	float pressure_val;
            	if(mission_mgr.getOpMode() == OPMODE_SIM)
				{
            		pressure_val = mission_mgr.getSimpData();
				}
            	else
            	{
            		sensors.updateBMP();
            		pressure_val = sensors.getPressure();
            	}

            	mission_mgr.update_alt_buffer(pressure_to_alt(pressure_val) - mission_mgr.getLaunchAlt());
				//glider_ekf_update_baro(pressure_to_alt(pressure_val) - mission_mgr.getLaunchAlt(), 1.0f);

				OperatingState next_state = update_state(sensors, mission_mgr, mission_mgr.getOpState());
				if(next_state != mission_mgr.getOpState())
				{
					sensors.EEPROM_updateState(next_state);
					mission_mgr.setOpState(next_state);
				}
            }

            if(bno_flag)
            {
				sensors.updateBMP();
				float pressure_val;
            	pressure_val = sensors.getPressure();
				glider_ekf_update_baro(pressure_to_alt(pressure_val) - mission_mgr.getLaunchAlt(), 1.0f);
            	bno_flag = 0;

            	if(sensors.BNO_dataReady())
				{
					sensors.updateBNO();
					uint32_t current_time = HAL_GetTick();
					float dt = (current_time - bno_update_timer) / 1000.0f;
					if(dt <= 0) dt = 0.02f; // sanity check
					bno_update_timer = current_time;
					float bno_quat[5];
					sensors.getGameRotationVector(bno_quat);
					float linear_accel[4];
					sensors.getLinearAccel(linear_accel);
					CPL_IMU_to_NED(linear_accel, bno_quat);
					float velocities[3];
					ekf_get_vel(velocities);
					//serial.sendInfoDataMsg("Vel before prediction: NED (%.1f, %.1f, %.1f) m/s", velocities[0], velocities[1], velocities[2]);
					glider_ekf_predict(linear_accel, dt);
					ekf_get_vel(velocities);
					//serial.sendInfoDataMsg("Vel after prediction: NED (%.1f, %.1f, %.1f) m/s", velocities[0], velocities[1], velocities[2]);
					glider_ekf_update_bno_quaternion(bno_quat, bno_quat[4]);
					ekf_get_vel(velocities);
					//serial.sendInfoDataMsg("Vel after BNO update: NED (%.1f, %.1f, %.1f) m/s", velocities[0], velocities[1], velocities[2]);
				}

				sensors.updateGPS(serial);
				if(sensors.GPS_dataReady())
				{
					sensors.GPS_dataReadyOff();
					ekf_gps_update(sensors.getGPS_lat(), sensors.getGPS_lon(), sensors.getGPS_alt(), sensors.getGPS_sog(), sensors.getGPS_cog(), sensors.getGPS_pdop());
					serial.sendInfoDataMsg("GPS Update: Lat=%.6f, Lon=%.6f, Alt=%.1f, SOG=%.1f m/s, COG=%.1f deg, PDOP=%.1f", sensors.getGPS_lat(), sensors.getGPS_lon(), sensors.getGPS_alt(), sensors.getGPS_sog(), sensors.getGPS_cog(), sensors.getGPS_pdop());

					// ── GPS position → NED ──────────────────────────────────────────────
					const double gps_lat = sensors.getGPS_lat();
					const double gps_lon = sensors.getGPS_lon();
					const float  gps_alt = sensors.getGPS_alt();  // MSL (m)
					float gps_ne[2];
					convert_gps_to_local_ned((float)gps_lat, (float)gps_lon, 0.f, gps_ne);
					// NED down = home_alt_m - GPS_alt_MSL (positive below home during descent)
					const float gps_d = home_alt_m - gps_alt;

					// ── Velocity from GPS position differencing ─────────────────────────
					const uint32_t gps_now = HAL_GetTick();
					if (gps_has_prev) {
						const float gps_dt = (gps_now - gps_prev_tick) / 1000.0f;
						if (gps_dt > 0.05f && gps_dt < 5.0f) {  // sanity: 50 ms – 5 s
							float prev_ne[2];
							convert_gps_to_local_ned((float)gps_prev_lat, (float)gps_prev_lon, 0.f, prev_ne);
							gps_vn = (gps_ne[0] - prev_ne[0]) / gps_dt;
							gps_ve = (gps_ne[1] - prev_ne[1]) / gps_dt;
							gps_vd = (gps_prev_alt - gps_alt)  / gps_dt;  // positive while descending
						}
					}
					gps_prev_lat  = gps_lat;
					gps_prev_lon  = gps_lon;
					gps_prev_alt  = gps_alt;
					gps_prev_tick = gps_now;
					gps_has_prev  = true;

					if(plan_done)
					{
						uint32_t now = HAL_GetTick();
						float dt = (now - ctrl_timer) / 1000.0f;
						if (dt <= 0.f) dt = 0.02f;
						ctrl_timer = now;

						// Position from GPS; velocity from GPS differencing; attitude from EKF
						rsx::State st;
						float q[4], rpy[3];
						ekf_get_quaternion(q);
						quat_to_rpy(q, rpy);
						st.n  = gps_ne[0]; st.e  = gps_ne[1]; st.d  = gps_d;
						st.vn = gps_vn;    st.ve  = gps_ve;    st.vd = gps_vd;
						st.roll = rpy[0];  st.pitch = rpy[1];  st.yaw = rpy[2];

						// --- guidance -> heading command ---
						float target_heading = 0.0f;
						if(mission_mgr.getFlightCtrl() == AUTONOMOUS)
						{
							rsx::HeadingCmd cmd = guidance.getHeading(st);
							if(cmd.valid && cmd.phase != rsx::Phase::Landed)
							{
								target_heading = cmd.heading;
								float speed = sqrtf(st.vn*st.vn + st.ve*st.ve + st.vd*st.vd);
								uint16_t aileron_pwm  = controller.update_roll_control(target_heading, st.yaw, st.roll, dt);
								uint16_t elevator_pwm = controller.update_pitch_control(5.0f, st.vd, speed, st.pitch, dt);
								sensors.writeAileronServoPPM(aileron_pwm);
								sensors.writeElevatorServoPPM(elevator_pwm);
							}
						}
						else
						{
							float speed = sqrtf(st.vn*st.vn + st.ve*st.ve + st.vd*st.vd);
							uint16_t aileron_pwm  = controller.update_roll_control(target_heading, st.yaw, st.roll, dt);
							uint16_t elevator_pwm = controller.update_pitch_control(5.0f, st.vd, speed, st.pitch, dt);
							sensors.writeAileronServoPPM(aileron_pwm);
							sensors.writeElevatorServoPPM(elevator_pwm);
						}

						guidance.replan(st);
					}
				}
            }

            if (!plan_done && mission_mgr.getOpState() == PROBE_RELEASE)
            {
                rsx::GuidanceParams gp;

                // ── Deploy position from GPS, attitude from EKF ───────────────────────
                float dep_ne[2];
                convert_gps_to_local_ned((float)sensors.getGPS_lat(),
                                          (float)sensors.getGPS_lon(),
                                          0.f, dep_ne);
                gp.start_n = dep_ne[0];                        // North offset from home (m)
                gp.start_e = dep_ne[1];                        // East  offset from home (m)
                gp.start_d = home_alt_m - sensors.getGPS_alt(); // NED down (m), positive below home

                // Heading: GPS-differenced velocity when moving; fall back to EKF yaw.
                float q[4], rpy[3];
                ekf_get_quaternion(q);
                quat_to_rpy(q, rpy);
                const float gh = hypotf(gps_vn, gps_ve);
                gp.start_heading = (gps_has_prev && gh > 2.0f) ? atan2f(gps_ve, gps_vn) : rpy[2];

                // ── Landing target — convert GPS lat/lon to local NED ─────────────────
                // Set the landing GPS coords via the "GPS" ground command before flight.
                float land_ne[2];
                convert_gps_to_local_ned(mission_mgr.get_landing_lat(),
                                         mission_mgr.get_landing_lon(),
                                         0.0f, land_ne);
                gp.land_n = land_ne[0];  // North offset from home (m)
                gp.land_e = land_ne[1];  // East  offset from home (m)
                gp.land_d = 0.0f;        // Down = 0 → land at the same altitude as home

                // ── Landing axis ──────────────────────────────────────────────────────
                // Pre-derived from the runway line equation (lon = -1.5561798*lat - 19.8879).
                // The planner resolves the 180° ambiguity (picks the better approach end).
                // Override at runtime via the "AXIS" command if needed (calls gps_bearing_rad).
                gp.landing_axis = kLandingAxisRad;

                // ── Glider aerodynamics ───────────────────────────────────────────────
                gp.glide_ratio = kGlideRatio;           // nominal L/D; updated in-flight by replan()

                // ── Path geometry ─────────────────────────────────────────────────────
                gp.approach_len    = kApproachLength;   // final straight leg length (m)
                gp.min_turn_radius = kMinTurnRadius;    // physical minimum turn radius (m)
                gp.max_radius      = kMaxSpiralRadius;  // maximum loiter spiral radius (m)

                // ── Competition field bounding box ────────────────────────────────────
                // Convert the 4 GPS corners to NED and take the axis-aligned bounding box.
                // The planner rejects any candidate path that exits this rectangle.
                {
                    float cn[2];
                    float bNmin = 1e9f, bNmax = -1e9f, bEmin = 1e9f, bEmax = -1e9f;
                    for (int i = 0; i < 4; ++i) {
                        convert_gps_to_local_ned(kFieldCorners[i][0], kFieldCorners[i][1], 0.f, cn);
                        if (cn[0] < bNmin) bNmin = cn[0];
                        if (cn[0] > bNmax) bNmax = cn[0];
                        if (cn[1] < bEmin) bEmin = cn[1];
                        if (cn[1] > bEmax) bEmax = cn[1];
                    }
                    gp.box_n_min = bNmin;
                    gp.box_n_max = bNmax;
                    gp.box_e_min = bEmin;
                    gp.box_e_max = bEmax;
                }

                // ── Tracker ───────────────────────────────────────────────────────────
                gp.lookahead_drop = kLookaheadDrop;     // carrot lookahead in altitude drop (m)

                guidance.setParams(gp);
                rsx::PlanStatus planStatus = guidance.plan();
                serial.sendInfoDataMsg("Guidance plan: status=%d land_dir=%.0f side=%d R=%.1f N=%d",
                    (int)planStatus,
                    guidance.landingDirection() * 180.f / 3.14159f,
                    guidance.spiralSide(),
                    guidance.spiralRadius(),
                    guidance.loops());
                plan_done = (planStatus != rsx::PlanStatus::Infeasible);
            }
        }

        HAL_TIM_Base_Stop_IT(&htim1);
        HAL_TIM_Base_Stop_IT(&htim8);

        mission_mgr.reset_params();
        mission_mgr.waitingForSimp();
        mission_mgr.enableLogfile();

        serial.sendInfoMsg("Transitioning back to IDLE mode...");
    }
}

OperatingState update_state(SensorManager &sensors, MissionManager &mgr, OperatingState current_state)
{
	OperatingState new_state = current_state;
	switch(current_state)
	{
		case LAUNCH_PAD: {
			float alt = mgr.calculate_median_alt();
			if(alt > ASCENT_ALT_THRESHOLD_M)
			{
				new_state = ASCENT;
			}
			break;
		}

		case ASCENT: {
			float alt = mgr.calculate_median_alt();
			if(mgr.update_max_alt(alt))
			{
				sensors.EEPROM_updateMaxAlt(alt);
			}
			bool threshold_check = mgr.get_max_alt() - alt > DESCENT_FALL_THRESHOLD_M;
			if(mgr.descent_trigger(threshold_check))
			{
				new_state = APOGEE;
			}
			break;
		}

		case APOGEE: {
			float alt = mgr.calculate_median_alt();
			if(mgr.update_max_alt(alt))
			{
				sensors.EEPROM_updateMaxAlt(alt);
			}
			if(mgr.is_apogee_packet_sent())
			{
				new_state = DESCENT;
			}
			break;
		}

		case DESCENT: {
			if(!mgr.nosecone_check() && mgr.calculate_median_alt() <= mgr.get_max_alt() * 0.82)
			{
				sensors.activate_nosecone_release();
				mgr.nosecone_rel();
				sensors.EEPROM_updateNoseconeRel();
				nosecone_rel__payload_rel_timer = HAL_GetTick();
			}

			if(!mgr.probe_check() && mgr.calculate_median_alt() <= mgr.get_max_alt() * 0.80 && (HAL_GetTick()-nosecone_rel__payload_rel_timer>=1000))
			{
				sensors.activate_probe_release();
				mgr.probe_rel();
				sensors.EEPROM_updateProbeRel();
				wing_servo_timer = HAL_GetTick();
				if(mgr.getOpMode() == OPMODE_FLIGHT)
				{
					egg_timer = HAL_GetTick();
				}
			}

			if(mgr.probe_check())
			{
				new_state = PROBE_RELEASE;
			}

			break;
		}

		case PROBE_RELEASE: {
			if(mgr.getOpMode() == OPMODE_SIM)
			{
				if((HAL_GetTick()-wing_servo_timer>=1000) && !mgr.wing_check())
				{
					sensors.activate_wing_deployment();
					mgr.wing_rel();
					sensors.EEPROM_updateWingRel();
				}
				if(mgr.wing_check())
				{
					if(mgr.calculate_median_alt() < EGG_ALT_THRESHOLD_M && !mgr.egg_check())
					{
						sensors.activate_egg_release();
						mgr.egg_rel();
						sensors.EEPROM_updateEggRel();
					}
				}
				if(mgr.egg_check())
				{
					new_state = PAYLOAD_RELEASE;
				}
			}
			else
			{
				if(HAL_GetTick()-egg_timer>=EGG_TIMING_BUDGET_MS && mgr.calculate_median_alt() < mgr.get_max_alt() * 0.78)
				{
					if(!mgr.wing_check())
					{
						sensors.activate_wing_deployment();
						mgr.wing_rel();
						sensors.EEPROM_updateWingRel();
					}
				}

				if(mgr.wing_check())
				{
					float alt = mgr.calculate_median_alt();
					if(alt < (EGG_ALT_THRESHOLD_M + mgr.getEggAlt()) && !mgr.egg_check())
					{
						sensors.activate_egg_release();
						mgr.egg_rel();
						sensors.EEPROM_updateEggRel();
					}
				}
				if(mgr.egg_check())
				{
					new_state = PAYLOAD_RELEASE;
				}
			}
			break;
		}

		case PAYLOAD_RELEASE: {
			bool threshold_check = mgr.calculate_median_alt() < LANDED_THRESHOLD_M;
			if(mgr.landed_trigger(threshold_check))
			{
				new_state = LANDED;
				HAL_TIM_Base_Stop_IT(&htim8);
			}
			break;
		}

		case LANDED: {
			break;
		}

		default: {
			break;
		}
	}

	return new_state;
}
