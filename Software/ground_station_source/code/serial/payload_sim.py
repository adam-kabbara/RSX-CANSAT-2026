import math
import random
import time

from PyQt6.QtCore import QByteArray, QObject, QTimer, pyqtSignal


class PayloadSim(QObject):
    readyRead = pyqtSignal()
    errorOccurred = pyqtSignal(str)

    SEA_LEVEL_PRESSURE_KPA = 101.325
    START_LATITUDE = 38.3760167
    START_LONGITUDE = -79.6078722

    ARM_TIME_S = 2.0
    TAKEOFF_TIME_S = 8.0
    CLIMBOUT_TIME_S = 10.0
    CRUISE_TIME_S = 22.0
    LANDING_TIME_S = 10.0
    CRUISE_ALTITUDE_M = 24.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self._is_open = False
        self._buffer = []
        self._team_id = 6767
        self._rng = random.Random(self._team_id)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._generate_telemetry)

        self.mode = "F"
        self.calibrated = False
        self.cmd_echo = "NONE"
        self.cam1_active = False
        self.cam2_active = False

        self._reset_flight_state("IDLE")

    def open(self, mode):
        del mode
        self._is_open = True
        self.timer.start(1000)
        return True

    def close(self):
        self._is_open = False
        self.timer.stop()
        return True

    def isOpen(self):
        return self._is_open

    def setBaudRate(self, baud):
        del baud

    def setPort(self, port_info):
        del port_info

    def portName(self):
        return "SIM_PORT"

    def write(self, data):
        """Intercept data sent from GSW to Payload."""
        if not self._is_open:
            return 0

        msg = data.decode().strip()
        self._handle_command(msg)
        return len(data)

    def readAll(self):
        if not self._buffer:
            return QByteArray()
        data_str = "\r".join(self._buffer) + "\r"
        self._buffer.clear()
        return QByteArray(data_str.encode())

    def canReadLine(self):
        return len(self._buffer) > 0

    def readLine(self):
        if self._buffer:
            msg = self._buffer.pop(0) + "\r"
            return QByteArray(msg.encode())
        return QByteArray()

    def _handle_command(self, cmd_str):
        parts = cmd_str.split(",")
        if len(parts) < 3 or parts[0] != "CMD":
            return

        op = parts[2]
        val = parts[3] if len(parts) > 3 else ""
        self.cmd_echo = f"{op}:{val}" if val else op

        response = ""

        if op == "CX":
            response = self._handle_transmit_command(val)
        elif op == "CAL":
            self.calibrated = True
            response = f"$ MSG:Launch altitude calibrated to {self.altitude:.1f} m"
        elif op == "MEC":
            response = self._handle_mechanism_command(val)
        elif op == "SIM":
            response = self._handle_sim_command(val)
        elif op == "SIMP":
            self._handle_simp_pressure(val)
            return
        elif op == "ST":
            response = f"$ MSG:TIME SOURCE SET TO {val}"
        elif op == "RST":
            self._reset_flight_state("IDLE")
            response = "$ MSG:PAYLOAD SIM RESET."
        elif op == "TEST":
            response = f"$ MSG:CANSAT IS ONLINE.{{{self.mode}|{self.state}}}"
        elif op == "LOG":
            response = "$ MSG:NO SIMULATED LOGFILE AVAILABLE."

        if response:
            self._buffer.append(response)
            self.readyRead.emit()

    def _handle_transmit_command(self, val):
        if val == "ON":
            if not self.calibrated and self.mode != "S":
                return "$E MSG:CANNOT START TELEMETRY BEFORE CALIBRATING ALTITUDE!"
            self._reset_flight_state("LAUNCH_PAD")
            self.transmitting = True
            return "$ MSG:STARTING TELEMETRY TRANSMISSION."

        if val == "OFF":
            self.transmitting = False
            self.state = "IDLE"
            return "$ MSG:ENDING PAYLOAD TRANSMISSION."

        return f"$E MSG:UNKNOWN CX OPTION {val}"

    def _handle_mechanism_command(self, val):
        if val == "CAMERA_STAT:X":
            return f"$ MSG:{self._camera_status_text()}"

        if val == "CAMERA1_STAT:X":
            return f"$ MSG:CAMERA1 {'ON' if self.cam1_active else 'OFF'}"

        if val == "CAMERA2_STAT:X":
            return f"$ MSG:CAMERA2 {'ON' if self.cam2_active else 'OFF'}"

        if val == "CAMERA1:X":
            self.cam1_active = not self.cam1_active
            return f"$ MSG:CAMERA1 {'ON' if self.cam1_active else 'OFF'}"

        if val == "CAMERA2:X":
            self.cam2_active = not self.cam2_active
            return f"$ MSG:CAMERA2 {'ON' if self.cam2_active else 'OFF'}"

        if val == "PROBE:X":
            self._aux_event_until = self.mission_time + 2.0
            return "$ MSG:PROBE RELEASE FORCED."

        if val == "PAYLOAD:X":
            self._aux_event_until = self.mission_time + 2.0
            return "$ MSG:PAYLOAD RELEASE FORCED."

        if val.startswith("SERVO"):
            return f"$ MSG:SERVO COMMAND ACCEPTED {val}"

        return f"$ MSG:UNKNOWN MEC OPTION {val}"

    def _handle_sim_command(self, val):
        if val == "ENABLE":
            self.mode = "S"
            return "$ MSG:SIMULATION MODE ENABLED."

        if val == "ACTIVATE":
            if self.mode != "S":
                return "$E MSG:ENABLE SIMULATION MODE BEFORE ACTIVATING."
            return "$ MSG:SIM_START"

        if val == "DISABLE":
            self.mode = "F"
            self._sim_pressure_kpa = None
            self._sim_altitude_m = None
            return "$ MSG:SIMULATION MODE DISABLED."

        return f"$E MSG:UNKNOWN SIM OPTION {val}"

    def _handle_simp_pressure(self, val):
        try:
            pressure_pa = float(val)
        except ValueError:
            return

        if pressure_pa <= 0:
            return

        self._sim_pressure_kpa = pressure_pa / 1000.0
        self._sim_altitude_m = self._pressure_pa_to_altitude_m(pressure_pa)

    def _reset_flight_state(self, state):
        now = time.monotonic()

        self.transmitting = False
        self.mission_time = 0.0
        self.launch_time = now
        self._last_update_time = now
        self.packet_count = 0
        self.state = state

        self.altitude = 0.0
        self.temperature = 24.0
        self.pressure = self.SEA_LEVEL_PRESSURE_KPA
        self.voltage = 12.6
        self.current = 0.0
        self.vertical_velocity = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        self.acceleration_x = 0.0
        self.acceleration_y = 0.0
        self.acceleration_z = 0.0
        self._previous_velocity_x = 0.0
        self._previous_velocity_y = 0.0
        self._previous_velocity_z = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

        self.latitude = self.START_LATITUDE
        self.longitude = self.START_LONGITUDE
        self.gps_altitude = 0.0
        self.gps_sats = 8
        self._east_m = 0.0
        self._north_m = 0.0

        self._aux_event_until = 0.0
        self._sim_pressure_kpa = None
        self._sim_altitude_m = None

    def _generate_telemetry(self):
        """Generate one coherent 1 Hz telemetry packet."""
        if not self.transmitting:
            return

        now = time.monotonic()
        dt = self._clamp(now - self._last_update_time, 0.1, 2.0)
        self._last_update_time = now
        self.mission_time = max(0.0, now - self.launch_time)
        self.packet_count += 1

        previous_altitude = self.altitude
        self._update_flight_state(previous_altitude)
        self.vertical_velocity = (self.altitude - previous_altitude) / dt
        self._update_gps(dt)
        self._update_environment()

        gyro_r, gyro_p, gyro_y = self._gyro_values()
        accel_r, accel_p, accel_y = self._accel_values()
        altitude = max(0.0, self.altitude + self._jitter(0.12))
        temperature = self.temperature + self._jitter(0.08)
        pressure = max(0.0, self.pressure + self._jitter(0.015))
        voltage = max(0.0, self.voltage + self._jitter(0.035))
        current = max(0.0, self.current + self._jitter(4.0))
        gyro_r += self._jitter(0.25)
        gyro_p += self._jitter(0.25)
        gyro_y += self._jitter(0.25)
        accel_r += self._jitter(0.08)
        accel_p += self._jitter(0.08)
        accel_y += self._jitter(0.08)
        gps_altitude = max(0.0, self.gps_altitude + self._jitter(0.20))
        latitude = self.latitude + self._jitter(0.0000010)
        longitude = self.longitude + self._jitter(0.0000010)
        current_time = time.strftime("%H:%M:%S", time.gmtime())
        mission_time_utc = time.strftime("%H:%M:%S", time.gmtime(self.mission_time))
        cam_status = self._camera_status_bits()
        flight_ctrl = self._flight_ctrl_mode()
        quat_w, quat_x, quat_y, quat_z = self._quaternion_values()

        telemetry = (
            f"{self._team_id},{mission_time_utc},{self.packet_count},{self.mode},{self.state},"
            f"{altitude:.1f},{temperature:.1f},{pressure:.2f},"
            f"{voltage:.1f},{current:.0f},"
            f"{gyro_r:.1f},{gyro_p:.1f},{gyro_y:.1f},"
            f"{accel_r:.1f},{accel_p:.1f},{accel_y:.1f},"
            f"{current_time},{gps_altitude:.1f},{latitude:.7f},{longitude:.7f},"
            f"{self.gps_sats},{self.cmd_echo},{cam_status},"
            f"{flight_ctrl},{quat_w:.4f},{quat_x:.4f},{quat_y:.4f},{quat_z:.4f},"
            f"{self.velocity_x:.2f},{self.velocity_y:.2f},{self.velocity_z:.2f},"
            f"{self.acceleration_x:.2f},{self.acceleration_y:.2f},{self.acceleration_z:.2f}"
        )

        self._buffer.append(telemetry)
        self.readyRead.emit()

    def _update_flight_state(self, previous_altitude):
        if self.mode == "S" and self._sim_altitude_m is not None:
            self._update_from_simp_pressure(previous_altitude)
            return

        t = self.mission_time
        takeoff_end = self.ARM_TIME_S + self.TAKEOFF_TIME_S
        landing_start = takeoff_end + self.CLIMBOUT_TIME_S + self.CRUISE_TIME_S
        landing_end = landing_start + self.LANDING_TIME_S

        if t < self.ARM_TIME_S:
            self.state = "LAUNCH_PAD"
            self.altitude = 0.0
            return

        if t < takeoff_end:
            ascent_t = t - self.ARM_TIME_S
            progress = self._clamp(ascent_t / self.TAKEOFF_TIME_S, 0.0, 1.0)
            progress = progress * progress * (3.0 - 2.0 * progress)
            self.state = "ASCENT"
            self.altitude = self.CRUISE_ALTITUDE_M * progress
            return

        if t < landing_start:
            self.state = "APOGEE"
            self.altitude = self.CRUISE_ALTITUDE_M + 0.35 * math.sin(t * 0.7)
            return

        if t < landing_end:
            landing_t = t - landing_start
            progress = self._clamp(landing_t / self.LANDING_TIME_S, 0.0, 1.0)
            progress = progress * progress * (3.0 - 2.0 * progress)
            self.state = "DESCENT"
            self.altitude = max(0.0, self.CRUISE_ALTITUDE_M * (1.0 - progress))
            return

        self.altitude = 0.0
        self.state = "LANDED"

    def _update_from_simp_pressure(self, previous_altitude):
        self.altitude = max(0.0, self._sim_altitude_m)

        if self.packet_count < 5:
            self.state = "LAUNCH_PAD"
            return

        if self.altitude <= 0.5 and self.packet_count > 8:
            self.state = "LANDED"
            return

        if self.altitude > previous_altitude + 0.15:
            self.state = "ASCENT"
        elif self.altitude < previous_altitude - 0.15:
            self._set_descent_state()
        else:
            self.state = "APOGEE"

    def _set_descent_state(self):
        if self.altitude <= 0.0:
            self.altitude = 0.0
            self.state = "LANDED"
            return

        self.state = "DESCENT"

    def _update_gps(self, dt):
        east_speed, north_speed = self._horizontal_speed()
        self.velocity_x = east_speed
        self.velocity_y = north_speed
        self.velocity_z = self.vertical_velocity
        self.acceleration_x = (self.velocity_x - self._previous_velocity_x) / dt
        self.acceleration_y = (self.velocity_y - self._previous_velocity_y) / dt
        self.acceleration_z = (self.velocity_z - self._previous_velocity_z) / dt
        self._previous_velocity_x = self.velocity_x
        self._previous_velocity_y = self.velocity_y
        self._previous_velocity_z = self.velocity_z

        self._east_m += east_speed * dt
        self._north_m += north_speed * dt

        lat_scale = 111_320.0
        lon_scale = lat_scale * math.cos(math.radians(self.START_LATITUDE))
        self.latitude = self.START_LATITUDE + self._north_m / lat_scale
        self.longitude = self.START_LONGITUDE + self._east_m / lon_scale
        self.gps_altitude = self.altitude + 0.3 * math.sin(self.packet_count * 0.43)
        self.gps_sats = int(self._clamp(8 + round(2 * math.sin(self.mission_time * 0.09)), 6, 11))

    def _horizontal_speed(self):
        if self.state in ("IDLE", "LAUNCH_PAD", "LANDED"):
            return (0.0, 0.0)

        if self.state == "ASCENT":
            return (
                1.7 + 0.18 * math.sin(self.mission_time * 0.9),
                0.20 * math.cos(self.mission_time * 0.7),
            )

        if self.state == "APOGEE":
            cruise_t = max(0.0, self.mission_time - self.ARM_TIME_S - self.TAKEOFF_TIME_S)
            return (
                4.2 + 0.45 * math.cos(cruise_t * 0.22),
                0.75 * math.sin(cruise_t * 0.22),
            )

        east_correction = self._clamp(-self._east_m * 0.06, -2.0, 2.0)
        north_correction = self._clamp(-self._north_m * 0.08, -1.2, 1.2)
        return (
            2.1 + east_correction + 0.18 * math.sin(self.mission_time * 1.1),
            north_correction + 0.14 * math.cos(self.mission_time * 0.8),
        )

    def _update_environment(self):
        if self.mode == "S" and self._sim_pressure_kpa is not None:
            self.pressure = self._sim_pressure_kpa
        else:
            noise = 0.02 * math.sin(self.packet_count * 0.7)
            self.pressure = self._pressure_kpa_for_altitude(self.altitude) + noise

        temp_noise = 0.15 * math.sin(self.packet_count * 0.31)
        self.temperature = 24.0 - 0.0065 * self.altitude + temp_noise

        camera_count = int(self.cam1_active) + int(self.cam2_active)
        aux_spike_ma = 260.0 if self.mission_time < self._aux_event_until else 0.0
        radio_tx_ma = 75.0
        avionics_ma = 220.0
        sensor_ma = 45.0
        camera_ma = 450.0 * camera_count

        if self.state == "LAUNCH_PAD":
            motor_ma = 240.0 + 45.0 * abs(math.sin(self.packet_count * 0.8))
        elif self.state == "ASCENT":
            motor_ma = 3_800.0 + 260.0 * math.sin(self.packet_count * 0.45)
        elif self.state == "APOGEE":
            motor_ma = 2_250.0 + 180.0 * math.sin(self.packet_count * 0.32)
        elif self.state == "DESCENT":
            motor_ma = 1_300.0 + 130.0 * math.sin(self.packet_count * 0.38)
        elif self.state == "LANDED":
            motor_ma = 0.0
        else:
            motor_ma = 180.0

        ripple_ma = 85.0 * math.sin(self.packet_count * 1.7)
        self.current = avionics_ma + sensor_ma + radio_tx_ma + motor_ma + camera_ma + aux_spike_ma + ripple_ma
        self.current = max(0.0, self.current)

        camera_sag = 0.08 * camera_count
        aux_sag = 0.03 if aux_spike_ma else 0.0
        motor_sag = 0.38 * min(1.0, motor_ma / 4_000.0)
        mission_drain = 0.002 * (self.mission_time / 10.0)
        self.voltage = max(10.8, 12.6 - mission_drain - camera_sag - aux_sag - motor_sag)
        self._update_attitude()

    def _gyro_values(self):
        t = self.mission_time
        aux_event = 1.0 if t < self._aux_event_until else 0.0

        if self.state == "LAUNCH_PAD":
            return (
                0.4 * math.sin(t * 5.0),
                0.4 * math.cos(t * 4.6),
                0.2 * math.sin(t * 2.0),
            )

        if self.state == "ASCENT":
            return (
                3.2 * math.sin(t * 1.4),
                2.6 * math.cos(t * 1.2),
                8.0 + 2.5 * math.sin(t * 0.7),
            )

        if self.state == "APOGEE":
            return (
                4.5 * math.sin(t * 1.6) + 4.0 * aux_event,
                3.8 * math.cos(t * 1.4) - 3.0 * aux_event,
                14.0 * math.sin(t * 0.45),
            )

        if self.state == "DESCENT":
            return (
                5.0 * math.sin(t * 1.3),
                4.4 * math.cos(t * 1.1),
                6.0 + 2.0 * math.sin(t * 0.5),
            )

        return (0.1 * math.sin(t), 0.1 * math.cos(t), 0.0)

    def _accel_values(self):
        t = self.mission_time
        vibration = 0.18 * math.sin(self.packet_count * 2.8)
        aux_event = 1.0 if t < self._aux_event_until else 0.0

        if self.state == "LAUNCH_PAD":
            return (
                0.08 * math.sin(t * 5.0),
                0.08 * math.cos(t * 4.4),
                9.8 + vibration,
            )

        if self.state == "ASCENT":
            takeoff_t = max(0.0, t - self.ARM_TIME_S)
            boost = 2.8 * max(0.0, 1.0 - (takeoff_t / self.TAKEOFF_TIME_S))
            return (
                0.35 * math.sin(t * 1.2),
                0.30 * math.cos(t * 1.0),
                9.8 + boost + vibration,
            )

        if self.state == "APOGEE":
            return (
                0.75 * math.sin(t * 0.8) + 0.9 * aux_event,
                0.65 * math.cos(t * 0.9) - 0.7 * aux_event,
                9.8 + 0.25 * math.sin(t * 2.1) + vibration,
            )

        if self.state == "DESCENT":
            return (
                0.45 * math.sin(t * 1.0),
                0.35 * math.cos(t * 1.2),
                9.1 + 0.25 * math.sin(t * 1.6) + vibration,
            )

        return (0.0, 0.0, 9.8 + vibration)

    def _update_attitude(self):
        airspeed = math.hypot(self.velocity_x, self.velocity_y)
        self.pitch_deg = self._clamp(self.vertical_velocity * 3.0, -18.0, 22.0)
        self.roll_deg = self._clamp(self.velocity_y * 8.0, -24.0, 24.0)
        if airspeed > 0.2:
            self.yaw_deg = math.degrees(math.atan2(self.velocity_y, self.velocity_x))
        elif self.state in ("IDLE", "LAUNCH_PAD", "LANDED"):
            self.yaw_deg = 0.0

    def _flight_ctrl_mode(self):
        return "AUTONOMOUS" if self.transmitting else "MANUAL"

    def _quaternion_values(self):
        roll = math.radians(self.roll_deg)
        pitch = math.radians(self.pitch_deg)
        yaw = math.radians(self.yaw_deg)

        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        return (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )

    def _camera_status_bits(self):
        return int(self.cam1_active) + (2 * int(self.cam2_active))

    def _camera_status_text(self):
        cam1 = "ON" if self.cam1_active else "OFF"
        cam2 = "ON" if self.cam2_active else "OFF"
        return f"CAMERA1 {cam1}; CAMERA2 {cam2}"

    @classmethod
    def _pressure_kpa_for_altitude(cls, altitude_m):
        ratio = max(0.0, 1.0 - altitude_m / 44_330.0)
        return cls.SEA_LEVEL_PRESSURE_KPA * (ratio ** 5.255)

    @classmethod
    def _pressure_pa_to_altitude_m(cls, pressure_pa):
        pressure_kpa = pressure_pa / 1000.0
        ratio = pressure_kpa / cls.SEA_LEVEL_PRESSURE_KPA
        return 44_330.0 * (1.0 - ratio ** (1.0 / 5.255))

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))

    def _jitter(self, amplitude):
        return self._rng.uniform(-amplitude, amplitude)
