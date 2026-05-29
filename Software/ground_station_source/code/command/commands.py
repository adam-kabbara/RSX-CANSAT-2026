"""
Formatting for all commands that are sent to payload
Connects to Command Group elements
"""

from datetime import datetime, timezone
import time
from serial.serial import SerialManager
from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QMessageBox

class Commands(QObject):

    print_signal = pyqtSignal(str)

    def __init__(self, serial: SerialManager, joystick=None, graph_ui=None, parent=None):
        super().__init__(parent)
        self._TEAM_ID = 1011
        self._serial   = serial
        self._joystick = joystick
        self._graph_ui = graph_ui

        # Joystick attitude accumulators (initialized so callbacks can run safely)
        self._joy_roll = 0.0
        self._joy_pitch = 0.0
        self._joy_yaw = 0.0

    def _cmd(self, op, val=None):
        if val is None:
            return f"CMD,{self._TEAM_ID},{op}"
        return f"CMD,{self._TEAM_ID},{op},{val}"

    def command__check_connection(self):
        if self._serial.send_data(self._cmd(op="TEST")):
            self.print_signal.emit("Sent test message")
    
    def command__send_time(self, time_id):
        if time_id:
            if self._serial.send_data(self._cmd(op="ST", val="GPS")):
                self.print_signal.emit("Sent GPS Set Time Command")
        else:
            utc_time = datetime.now(timezone.utc)
            time_str = utc_time.strftime("%H:%M:%S")
            if self._serial.send_data(self._cmd(op="ST", val=time_str)):
                self.print_signal.emit(f"Sent new mission time '{time_str}'")

    def command__restart(self):
        if self._serial.send_data(self._cmd(op="RST")):
            self.print_signal.emit("Sent restart signal")
                                    
    def command__write_servo(self, servo_id, servo_val):
        if servo_id == -1 or servo_val not in range(0, 181):
            self._serial.error_catch.emit("Enter a servo # and value first")
        elif self._serial.send_data(self._cmd(op="MEC", val=f"SERVO:{servo_id}|{servo_val}")):
            self.print_signal.emit(f"Sent command to program servo {servo_id} to {servo_val}")

    def command__set_dc(self, __dc_motor_val):
        if self._serial.send_data(self._cmd(op="MEC", val=f"DC:{__dc_motor_val}")):
            self.print_signal.emit(f"Sent command to program DC motor to {__dc_motor_val}")

    def command__custom(self, __custom_msg):
        if self._serial.send_data(__custom_msg):
            self.print_signal.emit(f"Sent custom message: {__custom_msg}")

    def command__cal(self, cal_type: str):
        if self._serial.send_data(self._cmd(op="CAL2", val=cal_type)):
            self.print_signal.emit(f"Sent {cal_type} calibration command")

    def command__set_gps(self, __gps_lat, __gps_lon, __gps_rad):
        if self._serial.send_data(self._cmd(op="GPS", val=(f"{__gps_lat}|{__gps_lon}|{__gps_rad}"))):
            self.print_signal.emit(f"Sent command to set GPS coordinates to LAT:{__gps_lat}, LON:{__gps_lon}, RAD:{__gps_rad}")

    def command__toggle_camera(self, camera_id):
        if self._serial.send_data(self._cmd(op="MEC", val=f"CAM:{camera_id}")):
            self.print_signal.emit(f"Sent Camera {camera_id} toggle command")

    def command__mec_release(self, mec_id):
        if self._serial.send_data(self._cmd(op="MEC", val=f"REL:{mec_id}")):
            self.print_signal.emit(f"Sent force {["NOSECONE release", "CPL release", "WING DEPLOYMENT", "EGG release"][mec_id]} command")

    def command__cam_status(self):
        if self._serial.send_data(self._cmd(op="MEC", val="CAMERA_STATUS:X")):
            self.print_signal.emit("Requesting CAMERA status")
       
    def command__sim_mode(self, mode: str):
        if self._serial.send_data(self._cmd(op="SIM", val=mode)):
            self.print_signal.emit(f"Sent simulation mode '{mode}'")

    def command__flight_ctrl(self, mode: str):
        if self._joystick is not None and not self._joystick.is_connected():
            self.error_catch.emit("ERROR: No joystick connected, cannot change flight mode")
            return
        if self._serial.send_data(self._cmd(op="FLIGHT_CTRL", val=mode)):
            self.print_signal.emit(f"Sent flight ctrl '{mode}'")

    def command__alt_cal(self):
        if self._serial.send_data(self._cmd(op="CAL")):
            self.print_signal.emit(f"Sent altitude calibration command")
    
    def command__get_log(self):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setWindowTitle("CONFIRM: REQUEST TRANSMISSION OF MISSION LOGFILE")
        msg_box.setText("THIS WILL BLOCK ALL OTHER PROCESSES UNTIL COMPLETE!!")
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        msg_box.setDefaultButton(QMessageBox.StandardButton.No)

        response = msg_box.exec()
        if response == QMessageBox.StandardButton.Yes:
         if self._serial.send_data(self._cmd(op="LOG")):
             self.print_signal.emit("Attempting to retreive log data...")

    def command__start_mission(self):
        if self._serial.send_data(self._cmd(op="CX", val="ON")):  
            self.print_signal.emit("SENT TRANSMISSION ON COMMAND")
            return 1
        else:
            return 0
    
    def command__end_mission(self):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setWindowTitle("CONFIRM: ENDING MISSION")
        msg_box.setText("Are you sure you want to end the mission?")
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        msg_box.setDefaultButton(QMessageBox.StandardButton.No)

        response = msg_box.exec()
        if response == QMessageBox.StandardButton.Yes:
            if self._serial.send_data(self._cmd(op="CX", val="OFF")):
                self.print_signal.emit("SENT TRANSMISSION OFF COMMAND")
                return 1
        return 0
    
       # ---- Joystick slots ----

    _BUTTON_MEC_MAP = {
        0: 3,   # Trigger         → mec_release(3)
        1: 2,   # Thumb/Side      → mec_release(2)
        2: 0,   # Top Left  (3)   → mec_release(0)
        3: 1,   # Top Right (4)   → mec_release(1)
        4: 0,   # Top Left  (5)   → mec_release(0)
        5: 1,   # Top Right (6)   → mec_release(1)
    }

    def _on_joystick_button(self, button: int):
        mec_id = self._BUTTON_MEC_MAP.get(button)
        if mec_id is not None:
            self.command__mec_release(mec_id)

    def _on_joy_roll(self, v: float):
        self._joy_roll = v * 45.0
        if self._graph_ui is not None:
            self._graph_ui.update_attitude(self._joy_roll, self._joy_pitch, self._joy_yaw)

    def _on_joy_pitch(self, v: float):
        self._joy_pitch = -v * 30.0
        if self._graph_ui is not None:
            self._graph_ui.update_attitude(self._joy_roll, self._joy_pitch, self._joy_yaw)

    def _on_joy_yaw(self, v: float):
        self._joy_yaw = (self._joy_yaw + v * 2.0) % 360.0
        if self._graph_ui is not None:
            self._graph_ui.update_attitude(self._joy_roll, self._joy_pitch, self._joy_yaw)

