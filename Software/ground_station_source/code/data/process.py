"""
Process telemetry data
"""
import csv
import os
import re
from dataclasses import dataclass, fields, asdict
from PyQt6.QtCore import QObject, pyqtSignal
from gui.graph_gui import GraphWindow
from .simp import SimpManager

current_telemetry_state = ""

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
    GYRO_R: float
    GYRO_P: float
    GYRO_Y: float
    ACCEL_R: float
    ACCEL_P: float
    ACCEL_Y: float
    GPS_TIME: str
    GPS_ALTITUDE: float
    GPS_LATITUDE: float
    GPS_LONGITUDE: float
    GPS_SATS: str
    CMD_ECHO: str
    CAM_STATUS: int
    PACKET_RECV: int

class DataProcessor(QObject):

    log_end_signal = pyqtSignal()
    log_begin_signal = pyqtSignal()
    file_error_signal = pyqtSignal(str)
    sat_error_signal = pyqtSignal(str)
    sat_resp_signal = pyqtSignal(str)
    telemetry_data_signal = pyqtSignal(object)

    def __init__(self, graph_ui: GraphWindow, simp: SimpManager, parent=None):

        super().__init__(parent)
        
        self._outfile       = None
        self._csv_file      = None
        self._csv_writer    = None
        self._write_to_log  = False
        self._graph_ui      = graph_ui
        self._simp          = simp
        self._csv_error_msg = ""

        self._csv_fields = [field.name for field in fields(TelemetryData)]

        try:
            file_path = os.path.join(os.path.dirname(__file__), '..')
            self.output_dir = os.path.join(file_path, 'output')
            os.makedirs(self.output_dir, exist_ok=True)

            self._csv_file = open(os.path.join(self.output_dir, 'telemetry_data.csv'), "w", newline="")
            self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=self._csv_fields)
            self._csv_writer.writeheader()
        except Exception as e:
            self._csv_file = None
            self._csv_error_msg = e

    def csv_check(self):
        if self._csv_file is None:
            return False
        else:
            return True
        
    def get_csv_error_msg(self):
        return self._csv_error_msg
    
    def open_logfile(self):
        self._outfile = open(os.path.join(self.output_dir, 'flight_logs.txt'), "wb")
        if self._outfile is not None:
            return True
        else:
            return False
    
    def close_logfile(self):
        if self._outfile is not None:
            if not self._outfile.closed:
                self._outfile.close()

    def close_csv(self):
        if self._csv_file is not None:
            if not self._csv_file.closed:
                self._csv_file.close()

    def reset_csv(self):
        if self._csv_file is not None:
            self._csv_file.seek(0)
            self._csv_file.truncate()

    def process_data(self, msg):
        if self._write_to_log:
            if self._outfile is not None:
                self._outfile.write((msg + "\n").encode('utf-8'))
            if "$LOGFILE:END" in msg:
                self._write_to_log = 0
                self.close_logfile()
                self.log_end_signal.emit()
        else:
            if not msg.strip():
                return
        
        # Message is a response
        if(msg.startswith('$')):

            # Start sending SIMP commands
            if "SIM_START" in msg:
                self._simp.simp_enable()
                return

            # Get logfile
            if "$LOGFILE:BEGIN" in msg:
                self.log_begin_signal.emit()
                if self.open_logfile():
                    self._outfile.write((msg + "\n").encode('utf-8'))
                    self._write_to_log = 1
                else:
                    self.file_error_signal.emit("ERROR: Logfile could not be opened! Wait and try again!")
                return
            
            if "CAMERA1 ON" in msg:
                self._graph_ui.update_camera1_status("ON")
            
            if "CAMERA2 ON" in msg:
                self._graph_ui.update_camera2_status("ON")
                
            if "CAMERA1 OFF" in msg:
                self._graph_ui.update_camera1_status("OFF")
            
            if "CAMERA2 OFF" in msg:
                self._graph_ui.update_camera2_status("OFF")

            row = {field: "" for field in self._csv_writer.fieldnames}
            row["CMD_ECHO"] = msg
            self._csv_writer.writerow(row)

            msg_text = re.search(':(.+)', msg).group(1)
            if msg_text is None:
                msg_text = "UNEXPECTED FORMAT:" + msg
            try:
                mission_info = re.search('{(.+?)}', msg_text).group(1)
            except AttributeError:
                mission_info = "NONE"
            if mission_info != "NONE":
                msg_text = re.sub(r'{.+?}', '', msg_text).strip()
                new_mode, new_state = mission_info.split('|')
                self._graph_ui.update_mode(new_mode)
                self._graph_ui.update_state(new_state)

            if msg.startswith("$E"):
                self.sat_error_signal.emit(f"{msg_text}")
            else:
                self.sat_resp_signal.emit(f"{msg_text}")
        else: # Message is telemetry data
            self.parse_telemetry_string(msg)

    def parse_telemetry_string(self, msg):

        self._graph_ui.update_packet_count()

        if msg is None or msg.strip().replace(',', '') == '':
            self._graph_ui.update_packet_label()
            return  # message is empty or only whitespace/commas
        
        data = self.extract_data_str(msg)
        self.telemetry_data_signal.emit(data)

        # Update graphs and live data values
        if data.ALTITUDE is not None:
            self._graph_ui.update_alt_graph(data.ALTITUDE)
        
        if data.TEMPERATURE is not None:
            self._graph_ui.update_temp(data.TEMPERATURE)

        if data.PRESSURE is not None:
            self._graph_ui.update_pressure(data.PRESSURE)
        
        if data.VOLTAGE is not None:
            self._graph_ui.update_volt_graph(data.VOLTAGE)

        if data.CURRENT is not None:
            self._graph_ui.update_current_graph(data.CURRENT)

        if data.GYRO_R is not None and data.GYRO_P is not None and data.GYRO_Y is not None:
            new_gyro_data = [data.GYRO_R, data.GYRO_P, data.GYRO_Y]
            self._graph_ui.update_gyro_graph(new_gyro_data)

        if data.ACCEL_R is not None and data.ACCEL_P is not None and data.ACCEL_Y is not None:
            new_accel_data = [data.ACCEL_R, data.ACCEL_P, data.ACCEL_Y]
            self._graph_ui.update_accel_graph(new_accel_data)
   
        if data.GPS_LATITUDE is not None and data.GPS_LONGITUDE is not None:
            self._graph_ui.update_gps_map(data.GPS_LATITUDE, data.GPS_LONGITUDE)

        if data.GPS_ALTITUDE is not None:
            self._graph_ui.update_gps_alt(data.GPS_ALTITUDE)
        
        if data.MISSION_TIME is not None:
            self._graph_ui.update_mission_time(data.MISSION_TIME)

        if data.PACKET_COUNT is not None:
            self._graph_ui.update_packets_sent(data.PACKET_COUNT)
            self._graph_ui.update_packet_label()

        if data.MODE is not None:
            if(data.MODE == "F"):
                self._graph_ui.update_mode("FLIGHT")
            elif(data.MODE == "S"):
                self._graph_ui.update_mode("SIM")

        if data.STATE is not None:
            self._graph_ui.update_state(data.STATE)

        if data.GPS_TIME is not None:
            self._graph_ui.update_gps_time(data.GPS_TIME)

        if data.GPS_SATS is not None:
            self._graph_ui.update_sats(data.GPS_SATS)

        if data.CMD_ECHO is not None:
            self._graph_ui.update_cmd_echo(data.CMD_ECHO)

        if data.CAM_STATUS is not None:
            # CAMERA1 status
            if data.CAM_STATUS == 3 or data.CAM_STATUS == 1:
                self._graph_ui.update_camera1_status("ON")
            else:
                self._graph_ui.update_camera1_status("OFF")
            
            # CAMERA2 status
            if data.CAM_STATUS == 3 or data.CAM_STATUS == 2:
                self._graph_ui.update_camera2_status("ON")
            else:
                self._graph_ui.update_camera2_status("OFF")

        data_dict = asdict(data)
        self._csv_writer.writerow(data_dict)
    
    def extract_data_str(self, msg: str) -> TelemetryData:

        fields = msg.split(',')

        telemetry_data = TelemetryData(
            TEAM_ID      = self._parse_int(self._field(fields, 0)),
            MISSION_TIME = self._field(fields, 1),
            PACKET_COUNT = self._field(fields, 2),
            MODE         = self._field(fields, 3),
            STATE        = self._field(fields, 4),
            ALTITUDE     = self._parse_float(self._field(fields, 5)),
            TEMPERATURE  = self._parse_float(self._field(fields, 6)),
            PRESSURE     = self._parse_float(self._field(fields, 7)),
            VOLTAGE      = self._parse_float(self._field(fields, 8)),
            CURRENT      = self._parse_float(self._field(fields, 9)),
            GYRO_R       = self._parse_float(self._field(fields, 10)),
            GYRO_P       = self._parse_float(self._field(fields, 11)),
            GYRO_Y       = self._parse_float(self._field(fields, 12)),
            ACCEL_R      = self._parse_float(self._field(fields, 13)),
            ACCEL_P      = self._parse_float(self._field(fields, 14)),
            ACCEL_Y      = self._parse_float(self._field(fields, 15)),
            GPS_TIME     = self._field(fields, 16),
            GPS_ALTITUDE = self._parse_float(self._field(fields, 17)),
            GPS_LATITUDE = self._parse_float(self._field(fields, 18)),
            GPS_LONGITUDE= self._parse_float(self._field(fields, 19)),
            GPS_SATS     = self._field(fields, 20),
            CMD_ECHO     = self._field(fields, 21),
            CAM_STATUS   = self._parse_int(self._field(fields, 22)),
            PACKET_RECV  = self._graph_ui.get_packet_count()
        )

        return telemetry_data

    def _field(self, fields: list[str], index: int):
        if index >= len(fields):
            return None
        value = fields[index].strip()
        if value == "":
            return None
        return value

    def _parse_float(self, value):
        if value is None:
            return None
        try:
            return float(value)
        except ValueError:
            return None

    def _parse_int(self, value):
        if value is None:
            return None
        try:
            return int(float(value))
        except ValueError:
            return None
