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

    def __init__(self, serial: SerialManager, parent=None):
        super().__init__(parent)
        self._TEAM_ID = 1011
        self._serial  = serial

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
        if servo_id == -1 or servo_val == -1:
            self.print_signal.emit("Enter a servo # and value first")
        elif self._serial.send_data(self._cmd(op="MEC", val=f"SERVO{servo_id}|{servo_val}")):
            self.update_gui_log(f"Sent command to program servo {servo_id} to {servo_val}")

    def command__toggle_camera(self, camera_id):
        if self._serial.send_data(self._cmd(op="MEC", val=f"{camera_id}:X")):
            self.print_signal.emit(f"Sent Camera {camera_id} toggle command")

    def command__probe_release(self):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setWindowTitle("CONFIRM")
        msg_box.setText("CONFIRM: SEND PROBE RELEASE COMMAND")
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        msg_box.setDefaultButton(QMessageBox.StandardButton.No)
        response = msg_box.exec()
        if response == QMessageBox.StandardButton.Yes:
            if self._serial.send_data(self._cmd(op="MEC", val="RELEASE:X")):
                self.print_signal.emit(f"Sent force probe release command")

    def command__cam_status(self):
        if self._serial.send_data(self._cmd(op="MEC", val="CAMERA1_STAT:X")):
            self.print_signal.emit("Requesting CAMERA1 status")
        time.sleep(1)
        if self._serial.send_data(self._cmd(op="MEC", val="CAMERA2_STAT:X")):
            self.print_signal.emit("Requesting CAMERA2 status")
    
    def command__sim_mode(self, mode: str):
        if self._serial.send_data(self._cmd(op="SIM", val=mode)):
            self.print_signal.emit(f"Sent simulation mode '{mode}'")

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
