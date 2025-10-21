"""
Process telemetry data
"""

from PyQt6.QtCore import QObject
import csv
import os
from dataclasses import dataclass, fields 

# Structure to store packet data
@dataclass(frozen=True)
class TelemetryData:
    TEAM_ID: int
    MISSION_TIME: str
    PACKET_COUNT: str
    MODE: str
    STATE: str
    ALTITUDE: float
    TEMPERATURE: float
    PRESSURE: float
    VOLTAGE: float
    CURRENT: float
    GYRO_R: int
    GYRO_P: int
    GYRO_Y: int
    ACCEL_R: int
    ACCEL_P: int
    ACCEL_Y: int
    GPS_TIME: str
    GPS_ALTITUDE: float
    GPS_LATITUDE: float
    GPS_LONGITUDE: float
    GPS_SATS: str
    CMD_ECHO: str
    CAM_STATUS: int
    PACKET_RECV: int

class DataProcessor(QObject):

    def __init__(self, parent=None):

        super().__init__(parent)
        
        self._outfile      = None
        self._csv_file     = None
        self._csv_writer   = None
        self._write_to_log = False

        self._csv_fields = [field.name for field in fields(TelemetryData)]

        os.makedirs("output", exist_ok=True)
    
    def open_logfile(self):
        self._outfile = open("output/flight_logs.txt", "wb")
        if self._outfile:
            return True
        else:
            return False
    
    def close_logfile(self):
        self._outfile.close()
        if self._outfile.closed:
            return False
        else:
            return True
        
    def open_csv(self):
        self._csv_file = open("output/telemetry_data.csv", "w", newline="")
        if self._csv_file is None:
            return False
        
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=self._csv_fields)
        self._csv_writer.writeheader()
        return True

    def close_csv(self):
        if self._csv_file is not None:
            if not self._csv_file.closed:
                self._csv_file.close()

    def reset_csv(self):
        self._csv_file.seek(0)
        self._csv_file.truncate()

    def process_data(self, msg):
        if self._write_to_log:
            self._outfile.write((msg + "\n").encode('utf-8'))
            if "$LOGFILE:END" in msg:
                self._write_to_log = False
                # TODO
                '''
                self.get_log_overlay.hide()
                self.__write_to_logfile = 0
                self.__outfile.close()
                self.update_gui_log("Finished uploading log data")
               '''
      #  else:


    def recv_data(self):
        while self.__serial.canReadLine():
            msg = self.__serial.readLine().data().decode().strip()
            if self.__write_to_logfile:
                self.__outfile.write((msg + "\n").encode('utf-8'))
                if "$LOGFILE:END" in msg:
                    self.get_log_overlay.hide()
                    self.__write_to_logfile = 0
                    self.__outfile.close()
                    self.update_gui_log("Finished uploading log data")
            else:
                self.__recveived_data = msg
                self.__data_received.emit()