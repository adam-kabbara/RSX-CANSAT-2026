"""
Handle simulated pressure data
"""

from PyQt6.QtCore import QObject, QTimer
from serial.serial import SerialManager
import os

class SimpManager(QObject):

    def __init__(self, serial: SerialManager, parent=None):

        super().__init__(parent)

        self._simp_file = None
        self._simp_data = []
        self._last_line = ""
        self._TEAM_ID = 1011
        self._simp_index = 0
        self._simp_active = False
        self._simp_timer = QTimer()
        self._simp_timer.timeout.connect(self.send_simp_data)
        self._serial = serial
        self._error_msg = ""

        try:
            file_path = os.path.join(os.path.dirname(__file__), '..')
            self.output_dir = os.path.join(file_path, 'media')
            os.makedirs(self.output_dir, exist_ok=True)

            self._simp_file = open(os.path.join(self.output_dir, 'team1011_sim_data.txt'), "r", newline="")

            for line in self._simp_file:
                if line.startswith("CMD"):
                    self._simp_data.append(line.strip())

        except Exception as e:
            self._simp_file = None
            self._error_msg = str(e)

    def simp_check(self):
        if self._simp_file is None:
            return False
        else:
            return True
        
    def get_error_msg(self):
        return self._error_msg
        
    def send_simp_data(self):
        if(self._simp_index < len(self._simp_data)):
            self._last_line = self._simp_data[self._simp_index]
            self._simp_index += 1

        self._serial.send_data(str(self._last_line))

    def simp_disable(self):
        self._simp_timer.stop()

    def simp_enable(self):
        self._simp_timer.start(1000)
        self._simp_active = True
    
    def simp_on(self):
        return self._simp_active