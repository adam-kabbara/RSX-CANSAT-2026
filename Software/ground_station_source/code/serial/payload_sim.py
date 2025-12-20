import time

from PyQt6.QtCore import QObject, pyqtSignal, QTimer, QByteArray


class PayloadSim(QObject):
    readyRead = pyqtSignal()
    errorOccurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._is_open = False
        self._buffer = []  # input buffer
        self._team_id = 6767

        # Simulation State
        self.mission_time = 0
        self.packet_count = 0
        self.mode = "F"  # F=Flight, S=Sim
        self.state = "IDLE"
        self.altitude = 0.0
        self.temperature = 25.0
        self.pressure = 1013.25
        self.voltage = 12.0
        self.transmitting = False
        self.calibrated = False
        self.cmd_echo = "NONE"

        # Track cameras individually
        self.cam1_active = False
        self.cam2_active = False

        # Telemetry Timer (1Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._generate_telemetry)

    def open(self, mode):
        self._is_open = True
        self.timer.start(1000)  # 1Hz
        print("PAYLOAD SIM: Connection Opened")
        return True

    def close(self):
        self._is_open = False
        self.timer.stop()
        print("PAYLOAD SIM: Connection Closed")
        return True

    def isOpen(self):
        return self._is_open

    def setBaudRate(self, baud):
        pass  # No-op for mock

    def setPort(self, port_info):
        pass  # No-op for mock

    def portName(self):
        return "SIM_PORT"

    def write(self, data):
        """Intercept data sent from GSW to Payload"""
        if not self._is_open:
            return 0

        msg = data.decode().strip()
        print(f"PAYLOAD SIM RECV: {msg}")
        self._handle_command(msg)
        return len(data)

    def canReadLine(self):
        return len(self._buffer) > 0

    def readLine(self):
        """Return the next message from the buffer"""
        if self._buffer:
            # Pop the first message and format as QByteArray
            msg = self._buffer.pop(0) + "\n"
            return QByteArray(msg.encode())
        return QByteArray()

    def _handle_command(self, cmd_str):
        """Logic to respond to commands"""
        parts = cmd_str.split(',')
        if len(parts) < 3 or parts[0] != "CMD":
            return

        op = parts[2]
        val = parts[3] if len(parts) > 3 else ""

        # Update Echo
        self.cmd_echo = f"{op}:{val}" if val else op

        response = ""

        if op == "CX":
            if val == "ON":
                if self.calibrated or self.mode == "S":
                    self.transmitting = True
                    self.state = "LAUNCH_PAD"
                    response = "$ MSG:STARTING TELEMETRY TRANSMISSION."
                else:
                    response = "$E MSG:CANNOT START TELEMETRY BEFORE CALIBRATING ALTITUDE!"
            elif val == "OFF":
                self.transmitting = False
                self.state = "IDLE"
                response = "$ MSG:ENDING PAYLOAD TRANSMISSION."

        elif op == "CAL":
            self.calibrated = True
            response = f"$ MSG:Launch Altitude calibrated to {self.altitude:.2f}"

        elif op == "MEC":
            # UPDATED CAMERA LOGIC
            if val == "CAMERA1_STAT:X":
                status = "ON" if self.cam1_active else "OFF"
                response = f"$ MSG:CAMERA1 {status}"
            elif val == "CAMERA2_STAT:X":
                status = "ON" if self.cam2_active else "OFF"
                response = f"$ MSG:CAMERA2 {status}"
            elif val == "CAMERA1:X":
                self.cam1_active = not self.cam1_active
                response = f"$ MSG:CAMERA 1 {self.cam1_active}"
            elif val == "CAMERA2:X":
                self.cam2_active = not self.cam2_active
                response = f"$ MSG:CAMERA 2 {self.cam1_active}"
            else:
                response = f"$ MSG:UNKNOWN MEC OPTION {val}"

        elif op == "SIM":
            if val == "ENABLE":
                self.mode = "S"
            elif val == "DISABLE":
                self.mode = "F"
            response = f"$ MSG:SIMULATION MODE {val}"

        elif op == "TEST":
            response = f"$ MSG:CANSAT IS ONLINE.{{{self.mode}|{self.state}}}"

        # Add response to buffer and trigger read
        if response:
            self._buffer.append(response)
            self.readyRead.emit()

    def _generate_telemetry(self):
        """Generates fake flight data"""
        if not self.transmitting:
            return

        self.packet_count += 1
        current_time = time.strftime("%H:%M:%S", time.gmtime())

        # Simple Physics Simulation
        if self.state == "ASCENT":
            self.altitude += 5.5
            if self.altitude > 150: self.state = "DESCENT"
        elif self.state == "DESCENT":
            self.altitude -= 2.0
            if self.altitude <= 0:
                self.altitude = 0
                self.state = "LANDED"
        elif self.state == "LAUNCH_PAD" and self.packet_count > 5:
            self.state = "ASCENT"

        # Format: TEAM_ID,TIME,PKT,MODE,STATE,ALT,TEMP,PRESS,VOLT,CURR,GYRO,ACCEL,GPS...
        # cam_int Removed
        telemetry = (
            f"{self._team_id},{current_time},{self.packet_count},{self.mode},{self.state},"
            f"{self.altitude:.1f},{self.temperature:.1f},{self.pressure:.2f},12.5,0.5,"
            f"0,0,0,0,0,1,{current_time},100.0,43.66,-79.40,8,"
            f"{self.cmd_echo}"
        )

        self._buffer.append(telemetry)
        self.readyRead.emit()