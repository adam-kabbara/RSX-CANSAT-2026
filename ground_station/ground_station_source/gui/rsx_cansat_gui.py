"""
GUI for real-time CANSAT data visualisation

Author: RSX
Version: 1.4

TODO:  
- graph should auto scroll?
- remove command log horizontal scrolling (idk how)
"""
import sys
import webbrowser
from datetime import datetime, timezone
from collections import deque
import numpy as np
from dataclasses import dataclass, fields
import re
import time
import csv
import pyqtgraph as pg
from pyqtgraph import mkPen
from enum import Enum
from PyQt6.QtSerialPort import QSerialPortInfo, QSerialPort
from PyQt6.QtCore import Qt, pyqtSignal, QIODevice, QTimer, QTime, pyqtSlot, QUrl
from PyQt6.QtGui import QFont, QIcon, QIntValidator, QColor, QPalette
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QWidget,
    QMessageBox,
    QLabel,
    QGridLayout,
    QGroupBox,
    QLineEdit,
    QVBoxLayout,
    QHBoxLayout,
    QComboBox,
    QSystemTrayIcon,
    QSizePolicy,
    QTabWidget,
    QFormLayout,
    QListWidget,
    QListWidgetItem,
    QAbstractItemView,
    QApplication,
)

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
    GYRO_R: int
    GYRO_P: int
    GYRO_Y: int
    ACCEL_R: int
    ACCEL_P: int
    ACCEL_Y: int
    MAG_R: int
    MAG_P: int
    MAG_Y: int
    AUTO_GYRO_ROTATION_RATE: int
    GPS_TIME: str
    GPS_ALTITUDE: float
    GPS_LATITUDE: float
    GPS_LONGITUDE: float
    GPS_SATS: str
    CMD_ECHO: str
    CAM_STATUS: int
    PACKET_RECV: int

    def to_dict(self):
        return {key: str(value) for key, value in self.__dict__.items()}

csv_fields = [field.name for field in fields(TelemetryData)]

# Base graph plotting system
# Initialize plots and set fonts/colors
class BaseDynamicPlotter:

    pen_color_list = [
        (255, 0, 0),   # Red
        (0, 255, 0),   # Green
        (0, 0, 255),   # Blue
        (255, 255, 0), # Yellow
        (255, 165, 0), # Orange
        (0, 255, 255), # Cyan
        (255, 0, 255)  # Magenta
    ]
    
    def __init__(self, plot, title, timewindow, x_unit, y_unit):
        self.timewindow = timewindow
        self.last_time = None
        self.base_line_color_idx = 0
        self.pen_line_size = 3

        font = QFont("Roboto Mono")
        font.setPointSize(14)
        font.setWeight(QFont.Weight.Bold)

        self.plt = plot
        self.plt.setTitle(f'<span style="font-family: Monospace; font-size:14pt; font-weight:bold;">{title}</span>')
        self.plt.showGrid(x=True, y=True)
        self.plt.getAxis('bottom').setStyle(tickFont=font)
        self.plt.getAxis('bottom').setLabel(f'<span style="font-family: Monospace; font-size:14pt; font-weight:bold;">{x_unit}</span>')
        self.plt.getAxis('left').setStyle(tickFont=font)
        self.plt.getAxis('left').setLabel(f'<span style="font-family: Monospace; font-size:14pt; font-weight:bold;">{y_unit}</span>')
    
    def get_pen_color(self, index):
        return mkPen(self.pen_color_list[index % len(self.pen_color_list)], width=self.pen_line_size)

    def reset_plot(self):
        raise NotImplementedError

    def update_plot(self, *args):
        raise NotImplementedError

# Plotting system for regular graphs with 1 line
class DynamicPlotter(BaseDynamicPlotter):

    def __init__(self, plot, title, timewindow, x_unit, y_unit):
        super().__init__(plot, title, timewindow, x_unit, y_unit)
        self.databuffer = deque([0.0] * timewindow, maxlen=timewindow)
        self.x = np.linspace(-timewindow, 0, timewindow)
        self.y = np.zeros(self.databuffer.maxlen, dtype=float)
        self.curve = self.plt.plot(self.x, self.y, pen=self.get_pen_color(self.base_line_color_idx))
        #self.plt.getViewBox().setLimits(xMin=-5, xMax=5000, minXRange=5, yMin=-10000, yMax=10000, minYRange=2)
        self.plt.setXRange(-20, 0)
    def update_plot(self, new_val):

        current_time = time.time()

        time_diff = (current_time - self.last_time) if self.last_time else 0
            
        self.last_time = current_time

        self.databuffer.append(new_val)
        self.y[:] = self.databuffer

        self.x = np.roll(self.x, -1)
        self.x[-1] = self.x[-2] + time_diff

        self.curve.setData(self.x, self.y)
        self.plt.setXRange(self.x[-1] - 50, self.x[-1])
    
    def reset_plot(self):
        self.databuffer = deque([0.0] * self.timewindow, maxlen=self.timewindow)
        self.x = np.linspace(-self.timewindow, 0, self.timewindow)
        self.y[:] = 0
        self.curve.setData(self.x, self.y)
        self.last_time = None

# Plotting system for graphs with multiple lines
class DynamicPlotter_MultiLine(BaseDynamicPlotter):
    def __init__(self, plot, title, timewindow, num_lines, x_unit, y_unit):
        super().__init__(plot, title, timewindow, x_unit, y_unit)
        self.num_lines = num_lines
        self.databuffer = [deque([0.0] * timewindow, maxlen=timewindow) for _ in range(num_lines)]
        self.x = np.linspace(-timewindow, 0, timewindow)
        self.y = np.zeros(shape=(self.num_lines, timewindow), dtype=float)
        self.plt.getViewBox().setLimits(xMin=-5, xMax=5000, minXRange=5, yMin=-10000, yMax=10000, minYRange=2)
        self.curve = [
            self.plt.plot(self.x, self.y[i], pen=self.get_pen_color(self.base_line_color_idx + i))
            for i in range(self.num_lines)
        ]

        label_names = ["R/X", "P/Y", "Y/Z"]
        self.labels = []

        for i in range(min(self.num_lines, 3)):
            pen = self.get_pen_color(self.base_line_color_idx + i)
            color = pen.color()  # Extract QColor from QPen
            label = pg.TextItem(label_names[i], anchor=(0, 0.5), color=color)
            self.labels.append(label)
            self.plt.addItem(label)

        self.last_time = None

    def update_plot(self, new_vals):

        current_time = time.time()
        time_diff = (current_time - self.last_time) if self.last_time else 0
        self.last_time = current_time

        for i in range(self.num_lines):
            if new_vals[i] is not None:
                self.databuffer[i].append(new_vals[i])
                self.y[i] = self.databuffer[i]

        self.x = np.roll(self.x, -1)
        self.x[-1] = self.x[-2] + time_diff

        for i in range(self.num_lines):
            self.curve[i].setData(self.x, self.y[i])

        # Update only the first 3 labels
        for i in range(min(self.num_lines, 3)):
            latest_x = self.x[-1]
            latest_y = self.y[i][-1]
            self.labels[i].setPos(latest_x, latest_y)
    
    def reset_plot(self):
        self.databuffer = [deque([0.0] * self.timewindow, maxlen=self.timewindow) for _ in range(self.num_lines)]
        self.x = np.linspace(-self.timewindow, 0, self.timewindow)
        self.y[:] = 0
        for i in range(self.num_lines):
            self.curve[i].setData(self.x, self.y[i])
        self.last_time = None

# Plotting system where both x and y axis require updates from data
class DynamicPlotter_2d(BaseDynamicPlotter):
    def __init__(self, plot, title, timewindow, x_unit, y_unit, init_x=0.0, init_y=0.0):
        super().__init__(plot, title, timewindow, x_unit, y_unit)
        self.databuffer_x = deque([init_x] * timewindow, maxlen=timewindow)
        self.databuffer_y = deque([init_y] * timewindow, maxlen=timewindow)
        self.x = np.full(timewindow, init_x, dtype=float)
        self.y = np.full(timewindow, init_y, dtype=float)

        self.curve = self.plt.plot(self.x, self.y, pen=self.get_pen_color(self.base_line_color_idx))

    def update_plot(self, new_val_x, new_val_y):

        self.databuffer_x.append(new_val_x)
        self.databuffer_y.append(new_val_y)
        self.x[:] = self.databuffer_x
        self.y[:] = self.databuffer_y

        self.curve.setData(self.x, self.y)
    
    def reset_plot(self):
        last_x = self.databuffer_x[-1] if self.databuffer_x else 0.0
        last_y = self.databuffer_y[-1] if self.databuffer_y else 0.0
        self.databuffer_x = deque([last_x] * self.timewindow, maxlen=self.timewindow)
        self.databuffer_y = deque([last_y] * self.timewindow, maxlen=self.timewindow)
        self.x[:] = last_x
        self.y[:] = last_y
        self.curve.setData(self.x, self.y)

class CommandButtonGroup(Enum):
    MAIN = 0
    MODE = 1
    ADVANCED = 2
    SENSORS = 3
    CONNECTION = 4
    TELEMETRY = 5

class GroundStationApp(QMainWindow):

    # Emit a signal when serial data is received
    __data_received = pyqtSignal()

    def __init__(self):

        super().__init__()
        
        self.__data_received.connect(self.process_data)

        # Define macros for some variables
        self.__CURRENT_CMD_WINDOW           = None
        self.__recveived_data               = "NONE"
        self.__available_ports              = None
        self.__cansat_mode                  = "FLIGHT"
        self.__PORT_SELECTED_INFO           = None
        self.__serial                       = QSerialPort()
        self.__TEAM_ID                      = 3114
        self.__packet_recv_count            = 0
        self.__packet_sent_count            = 0
        self.__graph_time_window            = 500
        self.__csv_file                     = None
        self.__csv_writer                   = None
        self.__outfile                      = None
        self.__write_to_logfile             = 0
        self.__serial.setBaudRate(57600)
        self.__serial.readyRead.connect(self.recv_data)
        self.__serial.errorOccurred.connect(self.handle_serial_error)
        self.simp_timer = QTimer()
        self.simp_timer.timeout.connect(self.send_simp_data)
        self.simp_data                      = []
        self.current_simp_idx               = 0
        self.__servo_id                     = -1
        self.__servo_val                    = -1
        self.__camera_id                    = "NONE"
        self.__set_time_id                  = 1
        self.__last_gyro_r                  = 0.0
        self.__last_gyro_p                  = 0.0
        self.__last_gyro_y                  = 0.0

        self.setWindowTitle("CANSAT Ground Station")
        self.setWindowIcon(QIcon('icon.png'))

        tray = QSystemTrayIcon()
        tray.setIcon(QIcon('icon.png'))
        tray.setVisible(True)
        tray.show()

        # ------ FONTS ------ #
        button_font = QFont()
        button_font.setPointSize(14)
        button_font.setWeight(QFont.Weight.Medium)
        command_status_font = QFont()
        command_status_font.setPointSize(14)
        command_status_font.setWeight(QFont.Weight.Medium)
        graph_sidebar_font = QFont()
        graph_sidebar_font.setPointSize(14)
        graph_sidebar_font.setWeight(QFont.Weight.DemiBold)
        credit_font = QFont("Courier New")
        credit_font.setPointSize(10)
        live_graph_data_font = QFont("Roboto Mono")
        live_graph_data_font.setPointSize(14)
        live_graph_field_font = QFont()
        live_graph_field_font.setPointSize(14)
        # ------ FONTS ------ #

        # CENTRAL WIDGET
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        grid_layout = QGridLayout(self.central_widget)
        grid_layout.setHorizontalSpacing(10)
        grid_layout.setVerticalSpacing(20)
        grid_layout.setRowStretch(0, 1)
        grid_layout.setRowStretch(1, 2)

        # ------ COMMANDS GROUP ------ #
        commands_group_box = QGroupBox()
        commands_group_box.setFixedHeight(300)
        commands_group_box.setFixedWidth(500)
        commands_layout = QVBoxLayout(commands_group_box)

        self.button_mode = QPushButton("CHANGE MODE")
        self.button_mode.setFont(button_font)
        self.button_mode.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.MODE))

        self.button_connection_group = QPushButton("CONNECTION")
        self.button_connection_group.setFont(button_font)
        self.button_connection_group.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.CONNECTION))

        self.button_connect = QPushButton("OPEN/CLOSE GROUND PORT")
        self.button_connect.setFont(button_font)
        self.button_connect.clicked.connect(self.open_close_port)
        self.button_connect.hide()

        self.button_telemetry = QPushButton("TELEMETRY")
        self.button_telemetry.setFont(button_font)
        self.button_telemetry.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.TELEMETRY))

        self.button_transmit_on = QPushButton("START MISSION")
        self.button_transmit_on.setFont(button_font)
        self.button_transmit_on.clicked.connect(lambda: self.toggle_transmission(1))
        self.button_transmit_on.hide()

        self.button_transmit_off = QPushButton("END MISSION")
        self.button_transmit_off.setFont(button_font)
        self.button_transmit_off.clicked.connect(lambda: self.toggle_transmission(0))
        self.button_transmit_off.hide()

        self.button_advanced = QPushButton("ADVANCED")
        self.button_advanced.setFont(button_font)
        self.button_advanced.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.ADVANCED))

        self.button_back = QPushButton("BACK")
        self.button_back.setFont(button_font)
        self.button_back.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.MAIN))
        self.button_back.hide()

        self.combo_select_port = QComboBox()
        self.combo_select_port.setPlaceholderText("SELECT PORT")
        self.combo_select_port.setFont(button_font)
        self.combo_select_port.activated.connect(self.port_selected)
        self.combo_select_port.hide()

        self.button_restart = QPushButton("RESTART PROCESSOR")
        self.button_restart.setFont(button_font)
        self.button_restart.clicked.connect(self.send_restart)
        self.button_restart.hide()

        set_time_box = QHBoxLayout()

        self.button_set_time = QPushButton("SET TIME")
        self.button_set_time.setFont(button_font)
        self.button_set_time.clicked.connect(self.send_time)
        self.button_set_time.hide()

        self.set_time_field = QComboBox()
        self.set_time_field.addItem("COMPUTER", 0)
        self.set_time_field.addItem("GPS", 1)
        self.set_time_field.setFont(button_font)
        self.set_time_field.activated.connect(self.set_time_field_edited)
        self.set_time_field.hide()

        set_time_box.addWidget(self.button_set_time)
        set_time_box.addWidget(self.set_time_field)

        self.button_show_map = QPushButton("SHOW MAP")
        self.button_show_map.setFont(button_font)
        self.button_show_map.clicked.connect(lambda: self.update_map_view(self.GPS_LAT, self.GPS_LONG))
        self.button_show_map.hide()

        self.button_reset_mission = QPushButton("CLEAR PLOTS, COMMAND LOG, CSV FILE")
        self.button_reset_mission.setFont(button_font)
        self.button_reset_mission.clicked.connect(self.reset_mission)
        self.button_reset_mission.hide()

        self.button_sim_mode_enable = QPushButton("SIM MODE ENABLE")
        self.button_sim_mode_enable.setFont(button_font)
        self.button_sim_mode_enable.clicked.connect(lambda: self.change_sim_mode("ENABLE"))
        self.button_sim_mode_enable.hide()

        self.button_sim_mode_activate = QPushButton("SIM MODE ACTIVATE")
        self.button_sim_mode_activate.setFont(button_font)
        self.button_sim_mode_activate.clicked.connect(lambda: self.change_sim_mode("ACTIVATE"))
        self.button_sim_mode_activate.hide()

        self.button_sim_mode_disable = QPushButton("SIM MODE DISABLE")
        self.button_sim_mode_disable.setFont(button_font)
        self.button_sim_mode_disable.clicked.connect(lambda: self.change_sim_mode("DISABLE"))
        self.button_sim_mode_disable.hide()

        self.button_refresh_ports = QPushButton("REFRESH PORTS")
        self.button_refresh_ports.setFont(button_font)
        self.button_refresh_ports.clicked.connect(lambda: self.refresh_ports(True))
        self.button_refresh_ports.hide()

        self.button_get_log_data = QPushButton("GET CANSAT LOG DATA")
        self.button_get_log_data.setFont(button_font)
        self.button_get_log_data.clicked.connect(self.get_log_data)
        self.button_get_log_data.hide()

        self.button_sensor_control = QPushButton("SENSOR CONTROL")
        self.button_sensor_control.setFont(button_font)
        self.button_sensor_control.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.SENSORS))

        self.button_altitude_cal = QPushButton("CALIBRATE ALTITUDE")
        self.button_altitude_cal.setFont(button_font)
        self.button_altitude_cal.clicked.connect(self.altitude_cal)
        self.button_altitude_cal.hide()

        self.button_test_connection = QPushButton("CHECK CONNECTION")
        self.button_test_connection.setFont(button_font)
        self.button_test_connection.clicked.connect(self.check_remote_connection)
        self.button_test_connection.hide()

        ### Program servo
        program_servo_box = QHBoxLayout()

        self.servo_id_field = QComboBox()
        self.servo_id_field.setPlaceholderText("SELECT SERVO")
        self.servo_id_field.addItem("Camera [CPL3] [F]", 0)
        self.servo_id_field.addItem("Gyro [CPL1] [F]", 2)
        self.servo_id_field.addItem("Release [CLP2] [F]", 1)
        self.servo_id_field.addItem("Gyro [Camera] [B]", 3)
        self.servo_id_field.setFont(button_font)
        self.servo_id_field.activated.connect(self.servo_id_edited)

        self.servo_val_field = QLineEdit()
        self.servo_val_field.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.servo_val_field.setMaxLength(3)
        self.servo_val_field.setStyleSheet("""
            QLineEdit {
                background-color: #f0f0f0;
                border: 1px solid #cccccc;
                border-radius: 10px;
                padding: 4px;
                font-size: 14px;
            }
            
            QLineEdit:focus {
                border: 1px solid #0078d4;
                background-color: #ffffff;
            }
        """)
        int_validator = QIntValidator(self)
        self.servo_val_field.setValidator(int_validator)
        self.servo_val_field.editingFinished.connect(self.servo_val_edited)

        self.program_servo_button = QPushButton(" PROGRAM SERVO ")
        self.program_servo_button.setFont(button_font)
        self.program_servo_button.clicked.connect(self.program_servo)
        self.program_servo_button.hide()

        program_servo_box.addWidget(self.program_servo_button)
        program_servo_box.addWidget(self.servo_id_field)
        program_servo_box.addWidget(self.servo_val_field)
        
        self.servo_id_field.hide()
        self.program_servo_button.hide()
        self.servo_val_field.hide()
        ### end program servo

        ### program camera
        program_camera_box = QHBoxLayout()

        self.camera_id_field = QComboBox()
        self.camera_id_field.setPlaceholderText("SELECT CAMERA")
        self.camera_id_field.addItem("CAMERA1")
        self.camera_id_field.addItem("CAMERA2")
        self.camera_id_field.setFont(button_font)
        self.camera_id_field.activated.connect(self.camera_id_edited)

        self.program_camera_button = QPushButton("TOGGLE CAMERA")
        self.program_camera_button.setFont(button_font)
        self.program_camera_button.clicked.connect(self.toggle_camera)

        program_camera_box.addWidget(self.program_camera_button)
        program_camera_box.addWidget(self.camera_id_field)
        
        self.camera_id_field.hide()
        self.program_camera_button.hide()
        ### end program camera

        self.probe_release_force = QPushButton("FORCE PROBE RELEASE")
        self.probe_release_force.setFont(button_font)
        self.probe_release_force.clicked.connect(self.force_probe_release)
        self.probe_release_force.hide()

        self.camera_status_button = QPushButton("GET CAMERA STATUS")
        self.camera_status_button.setFont(button_font)
        self.camera_status_button.clicked.connect(self.get_cam_status)
        self.camera_status_button.hide()

        self.team_id_field = QLineEdit()
        self.team_id_field.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.team_id_field.setMaxLength(9)
        self.team_id_field.setStyleSheet("""
            QLineEdit {
                background-color: #f0f0f0;
                border: 1px solid #cccccc;
                border-radius: 10px;
                padding: 4px;
                font-size: 14px;
            }
            
            QLineEdit:focus {
                border: 1px solid #0078d4;
                background-color: #ffffff;
            }
        """)
        int_validator = QIntValidator(self)
        self.team_id_field.setValidator(int_validator)
        self.team_id_field.editingFinished.connect(self.team_id_edited)
        self.team_id_field_info = QLabel("Change TEAM ID (ground station)")
        self.team_id_field_info.setFont(button_font)
        team_id_editing_box = QHBoxLayout()
        team_id_editing_box.addWidget(self.team_id_field_info)
        team_id_editing_box.addWidget(self.team_id_field)
        self.team_id_field_info.hide()
        self.team_id_field.hide()

        commands_layout.addWidget(self.button_connection_group)
        commands_layout.addWidget(self.combo_select_port)
        commands_layout.addWidget(self.button_connect)
        commands_layout.addWidget(self.button_refresh_ports)
        commands_layout.addWidget(self.button_test_connection)
        commands_layout.addWidget(self.button_telemetry)
        commands_layout.addWidget(self.button_transmit_on)
        commands_layout.addWidget(self.button_transmit_off)
        commands_layout.addWidget(self.button_restart)
        commands_layout.addWidget(self.button_sensor_control)
        commands_layout.addWidget(self.button_mode)
        commands_layout.addWidget(self.button_altitude_cal)
        commands_layout.addWidget(self.camera_status_button)
        commands_layout.addLayout(program_servo_box)
        commands_layout.addLayout(program_camera_box)
        commands_layout.addWidget(self.button_advanced)
        commands_layout.addLayout(set_time_box)
        commands_layout.addWidget(self.button_reset_mission)
        commands_layout.addWidget(self.button_show_map)
        commands_layout.addWidget(self.button_sim_mode_enable)
        commands_layout.addWidget(self.button_sim_mode_activate)
        commands_layout.addWidget(self.button_sim_mode_disable)
        commands_layout.addWidget(self.button_get_log_data)
        commands_layout.addWidget(self.probe_release_force)
        commands_layout.addLayout(team_id_editing_box)
        commands_layout.addWidget(self.button_back)

        grid_layout.setColumnStretch(0,1)

        grid_layout.addWidget(commands_group_box, 0, 0)

        # Store buttons in groups so we can control them later
        self.buttons_main = [
            self.button_advanced,
            self.button_connection_group,
            self.button_mode,
            self.button_sensor_control,
            self.button_telemetry,
        ]
        
        self.buttons_adv = [
            self.button_show_map,
            self.button_reset_mission,
            self.button_back,
            self.button_get_log_data,
            self.team_id_field,
            self.team_id_field_info,
        ]

        self.buttons_telemetry = [
            self.button_transmit_on,
            self.button_transmit_off,
            self.button_restart,
            self.button_back,
        ]

        self.buttons_mode = [
            self.button_sim_mode_enable,
            self.button_sim_mode_disable,
            self.button_sim_mode_activate,
            self.button_back,
        ]

        self.buttons_sensor = [
            self.button_set_time,
            self.set_time_field,
            self.button_back,
            self.button_altitude_cal,
            self.program_servo_button,
            self.servo_id_field,
            self.servo_val_field,
            self.program_camera_button,
            self.camera_id_field,
            self.camera_status_button,
            self.probe_release_force
        ]

        self.buttons_connection = [
            self.button_test_connection,
            self.button_back,
            self.button_connect,
            self.combo_select_port,
            self.button_refresh_ports,
        ]

        self.get_log_overlay = QLabel("Logfile collection in progress.", self)
        self.get_log_overlay.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.get_log_overlay.setStyleSheet("""
            background-color: rgba(0, 0, 0, 215);
            color: white;
            font-size: 18px;
        """)
        self.get_log_overlay.setGeometry(self.rect())
        self.get_log_overlay.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
        self.get_log_overlay.hide()
        # ------ END COMMANDS GROUP ------ #

        # ------ DATA GROUP ------ #
        self.GPS_LAT, self.GPS_LONG = None, None
        status_group_box = QGroupBox()
        status_layout = QVBoxLayout(status_group_box)

        self.label_port = QLabel()
        self.label_port.setFont(command_status_font)
        self.label_port.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.set_port_text_closed()

        self.label_remote_state = QLabel()
        self.label_remote_state.setFont(command_status_font)
        self.label_remote_state.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.label_remote_state.setText(f'<span style="color:black;">CANSAT State: \
                                              </span><span style="color:GREY;">N/A</span>')

        self.label_remote_mode = QLabel()
        self.label_remote_mode.setFont(command_status_font)
        self.label_remote_mode.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.label_remote_mode.setText(f'<span style="color:black;">CANSAT Mode: \
                                              </span><span style="color:GREY;">N/A</span>')
        
        self.label_mission_time = QLabel()
        self.label_mission_time.setFont(command_status_font)
        self.label_mission_time.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.label_mission_time.setText(f'<span style="color:black;">Mission Time: \
                                              </span><span style="color:GREY;">N/A</span>')

        self.label_packet_count = QLabel()
        self.label_packet_count.setFont(command_status_font)
        self.label_packet_count.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.label_packet_count.setText(f'<span style="color:black;">Packets Received/Sent: \
                                              </span><span style="color:GREY;">N/A</span>')
        
        self.label_sat = QLabel()
        self.label_sat.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.label_sat.setFont(command_status_font)
        self.label_sat.setText(f'<span style="color:black;">Satellites: \
                                              </span><span style="color:GREY;">N/A</span>')
        
        self.label_cmd_echo = QLabel()
        self.label_cmd_echo.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.label_cmd_echo.setFont(command_status_font)
        self.label_cmd_echo.setText(f'<span style="color:black;">CMD ECHO: \
                                              </span><span style="color:GREY;">N/A</span>')
        
        self.camera1_status_label = QLabel()
        self.camera1_status_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.camera1_status_label.setFont(command_status_font)
        self.camera1_status_label.setText(f'<span style="color:black;">CAMERA1 Status: \
                                              </span><span style="color:GREY;">N/A</span>')
        
        self.camera2_status_label = QLabel()
        self.camera2_status_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.camera2_status_label.setFont(command_status_font)
        self.camera2_status_label.setText(f'<span style="color:black;">CAMERA2 Status: \
                                              </span><span style="color:GREY;">N/A</span>')

        status_layout.addWidget(self.label_port)
        status_layout.addWidget(self.label_remote_mode)
        status_layout.addWidget(self.label_remote_state)
        status_layout.addWidget(self.label_mission_time)
        status_layout.addWidget(self.label_sat)
        status_layout.addWidget(self.label_packet_count)
        status_layout.addWidget(self.camera1_status_label)
        status_layout.addWidget(self.camera2_status_label)
        status_layout.addWidget(self.label_cmd_echo)

        grid_layout.setColumnStretch(1,1)

        grid_layout.addWidget(status_group_box, 0, 1)
        # ------ END DATA GROUP ------ #

        # ------  LOG GROUP ------ #
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)

        log_title = QLabel("Command Log")
        log_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        log_title.setFont(graph_sidebar_font)

        self.gui_log = QListWidget()
        self.gui_log.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.gui_log.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.gui_log.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.gui_log.setStyleSheet("""
            QListWidget {
                font-size: 18px;
                background-color: #dcdcdc;
                border-radius: 6px;
                padding: 3px;
            }
        """)

        log_layout.addWidget(log_title)
        log_layout.addWidget(self.gui_log)

        grid_layout.setColumnStretch(2,1)

        grid_layout.addWidget(log_widget, 0, 2)
        # ------ END LOG GROUP ------ #

        # ------ GRAPH GROUP ------ #
        graph_parent_group = QHBoxLayout()
        self.tab_widget = QTabWidget()
        graph_parent_group.addWidget(self.tab_widget, stretch=8)

        self.tab_widget.setStyleSheet("""
            QTabBar::tab {
                font-size: 14pt;
                padding: 4px 8px;
                background: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, 
                                    stop:0 rgba(255, 255, 255, 255), 
                                    stop:1 rgba(240, 240, 240, 255)); 
                border: 1px solid lightgray;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
            }
            QTabBar::tab:selected {
                background: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, 
                                    stop:0 rgba(240, 240, 240, 255), 
                                    stop:1 rgba(210, 210, 210, 255)); 
            }
        """)

        self.graphs = []
        self.plotters = []

        graph_info = [
            {"title": "Altitude", "lines": 1, "2d": False, "x_unit": "s", "y_unit": "m"},
            {"title": "Temperature", "lines": 1, "2d": False, "x_unit": "s", "y_unit": "°C"},
            {"title": "Pressure", "lines": 1, "2d": False, "x_unit": "s", "y_unit": "kPa"},
            {"title": "Voltage", "lines": 1, "2d": False, "x_unit": "s", "y_unit": "V"},
            {"title": "Gyro", "lines": 3, "2d": False, "x_unit": "s", "y_unit": "deg/s"},
            {"title": "Accel RPY ", "lines": 3, "2d": False, "x_unit": "s", "y_unit": "deg/s^2"},
            {"title": "Accel XYZ", "lines": 3, "2d": False, "x_unit": "s", "y_unit":"m/s^2"},
            {"title": "Magnetometer", "lines": 3, "2d": False, "x_unit": "s", "y_unit": "G"},
            {"title": "Rotation", "lines": 1, "2d": False, "x_unit": "s", "y_unit": "deg/s"},
            {"title": "GPS Lat v Long", "lines": 1, "2d": True, "x_unit": "Latitude", "y_unit": "Longitude"},
            {"title": "GPS Altitude", "lines": 1, "2d": False, "x_unit": "s", "y_unit": "m"}
        ]   
        
        self.graph_title_to_index = {
            "Altitude" : 0,
            "Temperature" : 1,
            "Pressure" : 2,
            "Voltage" : 3,
            "Gyro" : 4,
            "Gyro Diff": 5,
            "Accel" : 6,
            "Mag" : 7,
            "Rotation" : 8,
            "GPS" : 9,
            "GPS Altitude": 10,
        }

        # Loop through each graph and create a plot using the plot classes
        # Add the graph to a new tab and store plots for updating later
        for entry in graph_info:

            tab_content = QGroupBox()
            tab_layout = QVBoxLayout()

            graph = pg.PlotWidget()
            graph.setBackground('w')
            graph.setAlignment(Qt.AlignmentFlag.AlignCenter)
            tab_layout.addWidget(graph)

            tab_content.setLayout(tab_layout)

            self.tab_widget.addTab(tab_content, entry["title"])

            self.graphs.append(graph)

            if entry["lines"] == 1 and entry["2d"] is False:
                plotter = DynamicPlotter(graph, title=entry["title"], timewindow=self.__graph_time_window,x_unit=entry["x_unit"],y_unit=entry["y_unit"])
            elif entry["lines"] > 1 and entry["2d"] is False:
                plotter = DynamicPlotter_MultiLine(graph, title=entry["title"], 
                                                   timewindow=self.__graph_time_window, num_lines=entry["lines"],
                                                   x_unit=entry["x_unit"],y_unit=entry["y_unit"])
            else:
                init_lat = 38.149574
                init_long = 79.0737
                plotter = DynamicPlotter_2d(
                    graph, title=entry["title"], timewindow=self.__graph_time_window,
                    x_unit=entry["x_unit"], y_unit=entry["y_unit"],
                    init_x=init_lat, init_y=init_long
                )

            self.plotters.append(plotter)

        # Sidebar to show all current graph values
        sidebar_widget = QWidget()
        sidebar = QVBoxLayout(sidebar_widget)

        self.info_label = QLabel("Live Values")
        self.info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.info_label.setFont(graph_sidebar_font)

        self.credit_label = QLabel("RSX @ University of Toronto")
        self.credit_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.credit_label.setFont(credit_font)

        self.live_graph_values = QFormLayout()

        sidebar_fields_data = [
            ("Altitude", "0.0 m"),
            ("Temperature", "0.0 °C"),
            ("Pressure", "0.0 kPa"),
            ("Voltage", "0.0 V"),
            ("Gyro R", "0 °/s"),
            ("Gyro P", "0 °/s"),
            ("Gyro Y", "0 °/s"),
            ("Accel X", "0 m/s²"),
            ("Accel Y", "0 m/s²"),
            ("Accel Z", "0 m/s²"),
            ("RAccel R", "0 °/s²"),
            ("RAccel P", "0 °/s²"),
            ("RAccel Y", "0 °/s²"),
            ("Mag R", "0 G"),
            ("Mag P", "0 G"),
            ("Mag Y", "0 G"),
            ("Rotation", "0 °/s"),
            ("GPS Lat", "0.0000°"),
            ("GPS Long", "0.0000°"),
            ("GPS Altitude", "0.0 m"),
            ("GPS Time", "00:00:00"),
        ]

        self.sidebar_data_labels = []

        self.sidebar_data_dict = {name: idx for idx, (name, _) in enumerate(sidebar_fields_data)}

        for field_name, field_value in sidebar_fields_data:
            # Create the field label and data label
            field_label = QLabel(f"{field_name}:")
            data_label = QLabel(field_value)

            # Set fonts
            field_label.setFont(live_graph_field_font)
            data_label.setFont(live_graph_data_font)

            # Add them to your lists (or directly to your layout if needed)
            self.sidebar_data_labels.append(data_label)

            self.live_graph_values.addRow(field_label, data_label)

        form_group = QGroupBox()
        form_group.setLayout(self.live_graph_values)

        sidebar.addWidget(self.info_label)
        sidebar.addWidget(form_group)
        sidebar.addStretch()
        sidebar.addWidget(self.credit_label)

        graph_parent_group.addWidget(sidebar_widget, stretch=2)
        graph_parent_group.setSpacing(15)

        grid_layout.addLayout(graph_parent_group, 1, 0, 1, 3)
        # ------ END GRAPH GROUP ------ #

        # ------ START CSV FILE ------- #
        self.__csv_file = open("cansat_data_just_need_esp_files.csv", "w", newline="")
        self.__csv_writer = csv.DictWriter(self.__csv_file, fieldnames=csv_fields)
        self.__csv_writer.writeheader()
        # ------- END CSV FILE -------- #

        self.showMaximized()
    
    # ------ FUNCTIONS ------ #
    def update_map_view(self, lat, lon):
        try:
            # if self.GPS_LAT or self.GPS_LONG are not valid, open a not updated map thing
            if lat is None or lon is None or lat == 0.0 or lon == 0.0:
                self.update_gui_log("No GPS data yet", "red")
                return
            '''
            map_object = folium.Map(location=[lat, lon], zoom_start=15, prefer_canvas=True)
            folium.Marker([lat, lon], tooltip="CanSat Location").add_to(map_object)

            html = map_object.get_root().render()
            html = html.replace(
                    'https://unpkg.com/leaflet@1.7.1/dist/leaflet.css',
                    'leaflet/leaflet.css'
                ).replace(
                    'https://unpkg.com/leaflet@1.7.1/dist/leaflet.js',
                    'leaflet/leaflet.js'
                )
            
            with open("map.html", "w", encoding="utf-8") as f:
                f.write(html)
        
            webbrowser.open("map.html", new=2) '''
            webbrowser.open(f"https://www.google.com/maps/place/{lat},{lon}", new=2) # use google maps if there is data


        except Exception as e:
            self.update_gui_log(f"Map update failed: {e}", "red")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.get_log_overlay.setGeometry(self.rect())

    def update_gui_log(self, msg, color="black"):
        log_item = QListWidgetItem(f"{QTime.currentTime().toString('h:mm AP')}     {msg}")
        log_item.setForeground(QColor(color))
        self.gui_log.addItem(log_item)
        self.gui_log.scrollToBottom()

    # Change what buttons are shown in the commands box
    def command_group_change_buttons(self, mode):
        if mode == CommandButtonGroup.TELEMETRY:
            self.control_buttons(self.buttons_main, hide=True)
            self.control_buttons(self.buttons_telemetry)
            self.__CURRENT_CMD_WINDOW = CommandButtonGroup.TELEMETRY

        elif mode == CommandButtonGroup.MAIN:
            match self.__CURRENT_CMD_WINDOW:
                case CommandButtonGroup.ADVANCED:
                    self.control_buttons(self.buttons_adv, hide=True)
                case CommandButtonGroup.MODE:
                    self.control_buttons(self.buttons_mode, hide=True)
                case CommandButtonGroup.CONNECTION:
                    self.control_buttons(self.buttons_connection, hide=True)
                case CommandButtonGroup.SENSORS:
                    self.control_buttons(self.buttons_sensor, hide=True)
                case CommandButtonGroup.TELEMETRY:
                    self.control_buttons(self.buttons_telemetry, hide=True)
            self.control_buttons(self.buttons_main)
        
        elif mode == CommandButtonGroup.ADVANCED:
            self.control_buttons(self.buttons_main, hide=True)
            self.control_buttons(self.buttons_adv)
            self.__CURRENT_CMD_WINDOW = CommandButtonGroup.ADVANCED

        elif mode == CommandButtonGroup.MODE:
            self.control_buttons(self.buttons_main, hide=True)
            self.control_buttons(self.buttons_mode)
            self.__CURRENT_CMD_WINDOW = CommandButtonGroup.MODE
        
        elif mode == CommandButtonGroup.SENSORS:
            self.control_buttons(self.buttons_main, hide=True)
            self.control_buttons(self.buttons_sensor)
            self.__CURRENT_CMD_WINDOW = CommandButtonGroup.SENSORS
        
        elif mode == CommandButtonGroup.CONNECTION:
            self.combo_select_port.clear()
            self.combo_select_port.setPlaceholderText("SELECT PORT")
            self.refresh_ports(False)
            self.control_buttons(self.buttons_main, hide=True)
            self.control_buttons(self.buttons_connection)
            self.__CURRENT_CMD_WINDOW = CommandButtonGroup.CONNECTION
        
    def control_buttons(self, buttons, hide=False):
        for button in buttons:
            if hide:
                button.hide()
            else:
                button.show()
            
    # Refresh available ports connected to the computer
    def refresh_ports(self, b_print):
        self.combo_select_port.clear()
        self.combo_select_port.setPlaceholderText("SELECT PORT")
        self.__available_ports = QSerialPortInfo.availablePorts()
        for port in self.__available_ports:
            port_name = port.portName() + ": " + port.description()
            self.combo_select_port.addItem(port_name)
        if len(self.__available_ports) == 0:
            self.combo_select_port.addItem("No available ports")
        if(b_print == True):
            self.update_gui_log("Attempted port refresh")

    def port_selected(self):
        if len(self.__available_ports) != 0:
            self.__PORT_SELECTED_INFO = self.__available_ports[self.combo_select_port.currentIndex()]
            self.update_gui_log("Selected port: %s" % self.combo_select_port.currentText())
    
    # Open selected port or close it if it's open
    def open_close_port(self):
        if self.__serial.isOpen() is True:
            self.__serial.close()
            if self.__serial.isOpen():
                self.update_gui_log("ERROR: Could not close port!", "red")
            else:
                self.update_gui_log("Ground port was closed")
                self.set_port_text_closed()
        elif self.__PORT_SELECTED_INFO is not None:
            self.__serial.setPort(self.__PORT_SELECTED_INFO)
            if self.__serial.open(QIODevice.OpenModeFlag.ReadWrite):
                self.set_port_text_open()
                self.update_gui_log("Ground port opened")
            else:
                self.update_gui_log(f"FAILED to open port: {self.__PORT_SELECTED_INFO.portName()}!")
        else:
            self.update_gui_log("Select port before connecting!", "red")

    def check_remote_connection(self):
        if(self.send_data("CMD,%d,TEST,X" % self.__TEAM_ID)):
            self.update_gui_log("Sent test message")
    
    def send_time(self):
        if(self.__set_time_id):
           if(self.send_data("CMD,%d,ST,GPS" % (self.__TEAM_ID))):
                self.update_gui_log(f"Sent GPS Set Time Command") 
        else:
            utc_time = datetime.now(timezone.utc)
            time_str = utc_time.strftime("%H:%M:%S")
            if(self.send_data("CMD,%d,ST,%s" % (self.__TEAM_ID, time_str))):
                self.update_gui_log(f"Sent new mission time '{time_str}'")

    def send_restart(self):
        if(self.send_data("CMD,%d,RR,X" % self.__TEAM_ID)):
            self.update_gui_log("Sent restart signal")

    def program_servo(self):
        if(self.__servo_id == -1 or self.__servo_val == -1):
            self.update_gui_log("ERROR: Enter a servo # and value first!", "red")
        elif(self.send_data("CMD,%d,MEC,SERVO:%d|%d" % (self.__TEAM_ID, self.__servo_id, self.__servo_val))):
            servo_label = self.servo_id_field.itemText(self.servo_id_field.findData(self.__servo_id))
            self.update_gui_log(f"Sent command to program {servo_label} to {self.__servo_val}")

    def toggle_camera(self):
        if(self.send_data("CMD,%d,MEC,%s:X" % (self.__TEAM_ID, self.__camera_id))):
            self.update_gui_log(f"Sent {self.__camera_id} toggle command")
    
    def force_probe_release(self):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setWindowTitle("CONFIRM")
        msg_box.setText("CONFIRM: SEND PROBE RELEASE COMMAND")
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        msg_box.setDefaultButton(QMessageBox.StandardButton.No)
        response = msg_box.exec()
        if response == QMessageBox.StandardButton.Yes:
            if(self.send_data("CMD,%d,MEC,RELEASE:X" % self.__TEAM_ID)):
                self.update_gui_log(f"Sent force probe release command")

    def get_cam_status(self):
        if(self.send_data("CMD,%d,MEC,CAMERA1_STAT:X" % self.__TEAM_ID)):
            self.update_gui_log("Requesting CAMERA1 status")
        time.sleep(1)
        if(self.send_data("CMD,%d,MEC,CAMERA2_STAT:X" % self.__TEAM_ID)):
            self.update_gui_log("Requesting CAMERA2 status")


    def change_sim_mode(self, mode):
        if(self.send_data("CMD,%d,SIM,%s" % (self.__TEAM_ID, mode))):
            self.update_gui_log(f"Sent simulation mode '{mode}'")
    
    def altitude_cal(self):
        if(self.send_data("CMD,%d,CAL,X" % self.__TEAM_ID)):
            self.update_gui_log(f"Sent altitude calibration command")

    def get_log_data(self):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Icon.Warning    )
        msg_box.setWindowTitle("CONFIRM: REQUEST TRANSMISSION OF MISSION LOGFILE")
        msg_box.setText("THIS WILL BLOCK ALL OTHER PROCESSES UNTIL COMPLETE!!")
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        msg_box.setDefaultButton(QMessageBox.StandardButton.No)

        response = msg_box.exec()
        if response == QMessageBox.StandardButton.Yes:
            if(self.send_data("CMD,%d,GTLOGS,X" % self.__TEAM_ID)):
                self.update_gui_log("Attempting to retreive log data...")
        
    def toggle_transmission(self, toggle):
        if toggle:
            if(self.send_data("CMD,%d,CX,ON" % self.__TEAM_ID)):  
                self.update_gui_log("SENT TRANSMISSION ON COMMAND")
                self.__packet_recv_count = 0

                for plotter in self.plotters:
                    plotter.reset_plot()

        else:
            msg_box = QMessageBox()
            msg_box.setIcon(QMessageBox.Icon.Warning    )
            msg_box.setWindowTitle("CONFIRM: ENDING MISSION")
            msg_box.setText("Are you sure you want to end the mission?")
            msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            msg_box.setDefaultButton(QMessageBox.StandardButton.No)

            response = msg_box.exec()
            if response == QMessageBox.StandardButton.Yes:
                if(self.send_data("CMD,%d,CX,OFF" % self.__TEAM_ID)):
                    self.update_gui_log("SENT TRANSMISSION OFF COMMAND")
                    if(self.__cansat_mode == "SIM"):
                        self.simp_timer.stop()
                        self.current_simp_idx = 0

    def team_id_edited(self):
        self.team_id_field.clearFocus()
        self.__TEAM_ID = int(self.team_id_field.text())
        self.update_gui_log(f"Updated ground station TEAM ID to '{self.__TEAM_ID}'")
    
    def servo_id_edited(self, index):
        self.__servo_id = self.servo_id_field.itemData(index)
    
    def servo_val_edited(self):
        self.servo_val_field.clearFocus()
        self.__servo_val = int(self.servo_val_field.text())

    def camera_id_edited(self, index):
        self.__camera_id = self.camera_id_field.itemText(index)
    
    def set_time_field_edited(self, index):
        self.__set_time_id = self.set_time_field.itemData(index)

    @pyqtSlot(QSerialPort.SerialPortError)
    def handle_serial_error(self, error):
        if error == QSerialPort.SerialPortError.ResourceError:
            self.update_gui_log("SERIAL ERROR: Device disconnected", "red")
            self.__serial.close()
            self.set_port_text_closed()
        
        elif error == QSerialPort.SerialPortError.OpenError:
            self.update_gui_log("SERIAL ERROR: Could not open port", "red")

        elif error == QSerialPort.SerialPortError.DeviceNotFoundError:
            self.update_gui_log("SERIAL ERROR: Device not found", "red")
            self.__serial.close()
            self.set_port_text_closed()

        elif error != QSerialPort.SerialPortError.NoError:
            self.update_gui_log(f"SERIAL ERROR: {error} detected")

    def send_data(self, msg):
        if self.__serial.isOpen() is True:
            try:
                msg = msg + "\n"
                self.__serial.write(msg.encode())
                return 1
            except Exception as e:
                self.update_gui_log(f"ERROR: CANNOT SEND DATA - {e}", "red")
                self.__serial.close()
                self.set_port_text_closed()
        else:
            self.update_gui_log("ERROR: Open port before sending data!", "red")
            return 0
    
    def send_simp_data(self):
        if(self.current_simp_idx < len(self.simp_data)):
            line = self.simp_data[self.current_simp_idx]
            self.send_data(str(line))
            self.current_simp_idx += 1
        else:
            self.simp_timer.stop()

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

    @pyqtSlot()
    def process_data(self):
        # Info msg
        msg = self.__recveived_data

        if not msg.strip():
            return
        
        if(msg.startswith('$')):

            # Get logfile
            if "$LOGFILE:BEGIN" in msg:
                self.get_log_overlay.show()
                self.__outfile = open("cansat_logs.txt", "wb")
                self.__outfile.write((msg + "\n").encode('utf-8'))
                self.__write_to_logfile = 1
                return
            
            if "CAMERA1 ON" in msg:
                self.camera1_status_label.setText(f'<span style="color:black;">CAMERA1 Status: \
                                            </span><span style="color:GREEN;">ON</span>')
            
            if "CAMERA2 ON" in msg:
                self.camera2_status_label.setText(f'<span style="color:black;">CAMERA2 Status: \
                                            </span><span style="color:GREEN;">ON</span>')
                
            if "CAMERA1 OFF" in msg:
                self.camera1_status_label.setText(f'<span style="color:black;">CAMERA1 Status: \
                                            </span><span style="color:RED;">OFF</span>')
            
            if "CAMERA2 OFF" in msg:
                self.camera2_status_label.setText(f'<span style="color:black;">CAMERA2 Status: \
                                            </span><span style="color:RED;">OFF</span>')

            row = {field: "" for field in self.__csv_writer.fieldnames}
            row["CMD_ECHO"] = msg
            self.__csv_writer.writerow(row)

            msg_text = re.search('MSG:(.+)', msg).group(1)
            if msg_text is None:
                msg_text = "(UNEXPECTED FORMAT):" + msg
            try:
                mission_info = re.search('{(.+?)}', msg_text).group(1)
            except AttributeError:
                mission_info = "NONE"
            if mission_info != "NONE":
                msg_text = re.sub(r'{.+?}', '', msg_text).strip()
                new_mode, new_state = mission_info.split('|')
                self.__cansat_mode = new_mode
                self.label_remote_mode.setText(f'<span style="color:black;">CANSAT Mode: \
                                            </span><span style="color:BLUE;">{new_mode}</span>')
                self.label_remote_state.setText(f'<span style="color:black;">CANSAT State: \
                                              </span><span style="color:BLUE;">{new_state}</span>')

            if "BEGIN_SIMP" in msg:
                if(self.__cansat_mode == "SIM"):
                    try:
                        with open("cansat_2023_simp.txt", 'r') as file:
                            for line in file:
                                if line.startswith("CMD,$,SIMP"):
                                    line = line.replace('$', str(self.__TEAM_ID))
                                    self.simp_data.append(line.strip())
                        self.current_simp_idx = 0
                        self.simp_timer.start(1000)
                    except FileNotFoundError:
                        self.update_gui_log("ERROR: Could not find SIMP data file cansat_2023_simp.txt!", "red")

            if msg.startswith("$E"):
                self.update_gui_log(f"-> {msg_text}", "red")
            else:
                self.update_gui_log(f"-> {msg_text}", "blue")
        else: # telemetry
            self.parse_telemetry_string(msg)
    
    def reset_mission(self):     
        self.gui_log.clear()
        for plotter in self.plotters:
                plotter.reset_plot()
        self.__csv_file.seek(0)
        self.__csv_file.truncate()
        self.__packet_recv_count = 0
        self.__packet_sent_count = 0

    def set_port_text_closed(self):
         self.label_port.setText(f'<span style="color:black;">Ground Port: \
                                              </span><span style="color:RED;">CLOSED</span>')
        
    def set_port_text_open(self):
        open_msg = "OPEN ON: " + self.__PORT_SELECTED_INFO.portName() + self.__PORT_SELECTED_INFO.description()
        self.label_port.setText(f'<span style="color:black;">Ground Port: \
                                              </span><span style="color:GREEN;">{open_msg}</span>')

    # Close port on app exit
    def closeEvent(self, event):
        if self.__serial.isOpen() is True:
            self.__serial.close()
        if self.__csv_file is not None:
            if not self.__csv_file.closed:
                self.__csv_file.close()

    def update_packet_label(self):
        self.label_packet_count.setText(f'<span style="color:black;">Packets Received: \
                                            </span><span style="color:RED;"> \
                                            {self.__packet_recv_count}/{self.__packet_sent_count}</span>')
    
    # Upon receiving telemetry string, extract contents and update fields
    def parse_telemetry_string(self, msg):

        self.__packet_recv_count += 1
        self.update_packet_label()

        if msg is None or msg.strip().replace(',', '') == '':
            return  # message is empty or only whitespace/commas
        
        data = self.extract_data_str(msg)

        # Update graphs and live data values
        if data.ALTITUDE is not None:
            self.plotters[self.graph_title_to_index.get("Altitude")].update_plot(data.ALTITUDE)
            self.sidebar_data_labels[self.sidebar_data_dict.get("Altitude")].setText(f"{data.ALTITUDE} m")
        
        if data.TEMPERATURE is not None:
            self.plotters[self.graph_title_to_index.get("Temperature")].update_plot(data.TEMPERATURE)
            self.sidebar_data_labels[self.sidebar_data_dict.get("Temperature")].setText(f"{data.TEMPERATURE} °C")

        if data.PRESSURE is not None:
            self.plotters[self.graph_title_to_index.get("Pressure")].update_plot(data.PRESSURE)
            self.sidebar_data_labels[self.sidebar_data_dict.get("Pressure")].setText(f"{data.PRESSURE} kPa")
        
        if data.VOLTAGE is not None:
            self.plotters[self.graph_title_to_index.get("Voltage")].update_plot(data.VOLTAGE)
            self.sidebar_data_labels[self.sidebar_data_dict.get("Voltage")].setText(f"{data.VOLTAGE} V")

        new_gyro_data = [data.GYRO_R, data.GYRO_P, data.GYRO_Y]
        self.plotters[self.graph_title_to_index.get("Gyro")].update_plot(new_gyro_data)
        self.sidebar_data_labels[self.sidebar_data_dict.get("Gyro R")].setText(f"{data.GYRO_R} °/s")
        self.sidebar_data_labels[self.sidebar_data_dict.get("Gyro P")].setText(f"{data.GYRO_P} °/s")
        self.sidebar_data_labels[self.sidebar_data_dict.get("Gyro Y")].setText(f"{data.GYRO_Y} °/s")
        gyro_diff_data = [data.GYRO_R - self.__last_gyro_r, data.GYRO_P - self.__last_gyro_p, data.GYRO_Y - self.__last_gyro_y]
        self.plotters[self.graph_title_to_index.get("Gyro Diff")].update_plot(gyro_diff_data)
        self.__last_gyro_r = data.GYRO_R
        self.__last_gyro_p = data.GYRO_P
        self.__last_gyro_y = data.GYRO_Y
        self.sidebar_data_labels[self.sidebar_data_dict.get("RAccel R")].setText(f"{gyro_diff_data[0]} °/s²")
        self.sidebar_data_labels[self.sidebar_data_dict.get("RAccel P")].setText(f"{gyro_diff_data[1]} °/s²")
        self.sidebar_data_labels[self.sidebar_data_dict.get("RAccel Y")].setText(f"{gyro_diff_data[2]} °/s²")

        new_accel_data = [data.ACCEL_R, data.ACCEL_P, data.ACCEL_Y]
        self.plotters[self.graph_title_to_index.get("Accel")].update_plot(new_accel_data)
        self.sidebar_data_labels[self.sidebar_data_dict.get("Accel X")].setText(f"{data.ACCEL_R} m/s²")
        self.sidebar_data_labels[self.sidebar_data_dict.get("Accel Y")].setText(f"{data.ACCEL_P} m/s²")
        self.sidebar_data_labels[self.sidebar_data_dict.get("Accel Z")].setText(f"{data.ACCEL_Y} m/s²")

        new_mag_data = [data.MAG_R, data.MAG_P, data.MAG_Y]
        self.plotters[self.graph_title_to_index.get("Mag")].update_plot(new_mag_data)
        self.sidebar_data_labels[self.sidebar_data_dict.get("Mag R")].setText(f"{data.MAG_R} G")
        self.sidebar_data_labels[self.sidebar_data_dict.get("Mag P")].setText(f"{data.MAG_P} G")
        self.sidebar_data_labels[self.sidebar_data_dict.get("Mag Y")].setText(f"{data.MAG_Y} G")
        
        if data.AUTO_GYRO_ROTATION_RATE is not None:
            self.plotters[self.graph_title_to_index.get("Rotation")].update_plot(data.AUTO_GYRO_ROTATION_RATE)
            self.sidebar_data_labels[self.sidebar_data_dict.get("Rotation")].setText(f"{data.AUTO_GYRO_ROTATION_RATE} °/s")

        if data.GPS_LATITUDE is not None and data.GPS_LONGITUDE is not None:
            self.plotters[self.graph_title_to_index.get("GPS")].update_plot(data.GPS_LATITUDE, data.GPS_LONGITUDE)
            self.sidebar_data_labels[self.sidebar_data_dict.get("GPS Lat")].setText(f"{data.GPS_LATITUDE}°")
            self.sidebar_data_labels[self.sidebar_data_dict.get("GPS Long")].setText(f"{data.GPS_LONGITUDE}°")
            self.GPS_LAT, self.GPS_LONG = data.GPS_LATITUDE, data.GPS_LONGITUDE
        
        if data.GPS_ALTITUDE is not None:
            self.plotters[self.graph_title_to_index.get("GPS Altitude")].update_plot(data.GPS_ALTITUDE)
            self.sidebar_data_labels[self.sidebar_data_dict.get("GPS Altitude")].setText(f"{data.GPS_ALTITUDE} m")
        
        if data.MISSION_TIME is not None:
            self.label_mission_time.setText(f'<span style="color:black;">Mission Time: \
                                                </span><span style="color:BLUE;">{data.MISSION_TIME}</span>')
        if data.PACKET_COUNT is not None:
            self.__packet_sent_count = data.PACKET_COUNT
            self.update_packet_label()

        if data.MODE is not None:
            if(data.MODE == "F"):
                self.label_remote_mode.setText(f'<span style="color:black;">CANSAT Mode: \
                                                </span><span style="color:BLUE;">FLIGHT</span>')
            elif(data.MODE == "S"):
                self.label_remote_mode.setText(f'<span style="color:black;">CANSAT Mode: \
                                                </span><span style="color:BLUE;">SIM</span>')
        if data.STATE is not None:
            self.label_remote_state.setText(f'<span style="color:black;">CANSAT State: \
                                              </span><span style="color:BLUE;">{data.STATE}</span>')
        if data.GPS_TIME is not None:
            self.sidebar_data_labels[self.sidebar_data_dict.get("GPS Time")].setText(f"{data.GPS_TIME}")

        if data.GPS_SATS is not None:
            self.label_sat.setText(f'<span style="color:black;">Satellites: \
                                              </span><span style="color:BLUE;">{data.GPS_SATS}</span>')
        if data.CMD_ECHO is not None:
            self.label_cmd_echo.setText(f'<span style="color:black;">CMD ECHO: \
                                              </span><span style="color:RED;">{data.CMD_ECHO}</span>')

        if data.CAM_STATUS is not None:
            # CAMERA1 status
            if data.CAM_STATUS == 3 or data.CAM_STATUS == 1:
                self.camera1_status_label.setText(f'<span style="color:black;">CAMERA1 Status: \
                                            </span><span style="color:GREEN;">ON</span>')
            else:
                self.camera1_status_label.setText(f'<span style="color:black;">CAMERA1 Status: \
                                            </span><span style="color:RED;">OFF</span>')
            
            # CAMERA2 status
            if data.CAM_STATUS == 3 or data.CAM_STATUS == 2:
                self.camera2_status_label.setText(f'<span style="color:black;">CAMERA2 Status: \
                                            </span><span style="color:GREEN;">ON</span>')
            else:
                self.camera2_status_label.setText(f'<span style="color:black;">CAMERA2 Status: \
                                            </span><span style="color:RED;">OFF</span>')

            
        data_dict = data.to_dict()
        self.__csv_writer.writerow(data_dict)
    
    def extract_data_str(self, msg: str) -> TelemetryData:
        # EXPECTED FORMAT:
        # "TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE, ALTITUDE, TEMPERATURE, PRESSURE, 
        # VOLTAGE, GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y, MAG_R, MAG_P, MAG_Y, AUTO_GYRO_ROTATION_RATE, 
        # GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS, CMD_ECHO"

        fields = msg.split(',')

        telemetry_data = TelemetryData(
            TEAM_ID      = int(fields[0]) if fields else None,
            MISSION_TIME = fields[1] if 1 < len(fields) else None,
            PACKET_COUNT = fields[2] if 2 < len(fields) else None,
            MODE         = fields[3] if 3 < len(fields) else None,
            STATE        = fields[4] if 4 < len(fields) else None,
            ALTITUDE     = float(fields[5]) if 5 < len(fields) else None,
            TEMPERATURE  = float(fields[6]) if 6 < len(fields) else None,
            PRESSURE     = float(fields[7]) if 7 < len(fields) else None,
            VOLTAGE      = float(fields[8]) if 8 < len(fields) else None,
            GYRO_R       = int(fields[9]) if 9 < len(fields) else None,
            GYRO_P       = int(fields[10]) if 10 < len(fields) else None,
            GYRO_Y       = int(fields[11]) if 11 < len(fields) else None,
            ACCEL_R      = int(fields[12]) if 12 < len(fields) else None,
            ACCEL_P      = int(fields[13]) if 13 < len(fields) else None,
            ACCEL_Y      = int(fields[14]) if 14 < len(fields) else None,
            MAG_R        = float(fields[15]) if 15 < len(fields) else None,
            MAG_P        = float(fields[16]) if 16 < len(fields) else None,
            MAG_Y        = float(fields[17]) if 17 < len(fields) else None,
            AUTO_GYRO_ROTATION_RATE = float(fields[18]) if 18 < len(fields) else None,
            GPS_TIME     = fields[19] if 19 < len(fields) else None,
            GPS_ALTITUDE = float(fields[20]) if 20 < len(fields) else None,
            GPS_LATITUDE = float(fields[21]) if 21 < len(fields) else None,
            GPS_LONGITUDE= float(fields[22]) if 22 < len(fields) else None,
            GPS_SATS     = fields[23] if 23 < len(fields) else None,
            CMD_ECHO     = fields[24] if 24 < len(fields) else None,
            CAM_STATUS   = fields[25] if 25 < len(fields) else None,
            PACKET_RECV  = self.__packet_recv_count
        )

        return telemetry_data

def customPalette():

    palette = QPalette()

    palette.setColor(QPalette.ColorRole.WindowText, QColor("#000000"))
    palette.setColor(QPalette.ColorRole.Button, QColor("#f0f0f0"))
    palette.setColor(QPalette.ColorRole.Light, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.Midlight, QColor("#e3e3e3"))
    palette.setColor(QPalette.ColorRole.Dark, QColor("#a0a0a0"))
    palette.setColor(QPalette.ColorRole.Mid, QColor("#a0a0a0"))
    palette.setColor(QPalette.ColorRole.Text, QColor("#000000"))
    palette.setColor(QPalette.ColorRole.BrightText, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor("#000000"))
    palette.setColor(QPalette.ColorRole.Base, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.Window, QColor("#f0f0f0"))
    palette.setColor(QPalette.ColorRole.Shadow, QColor("#696969"))
    palette.setColor(QPalette.ColorRole.Highlight, QColor("#0078d7"))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.Link, QColor("#006770"))
    palette.setColor(QPalette.ColorRole.LinkVisited, QColor("#00343b"))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor("#e9e7e3"))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor("#ffffdc"))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor("#000000"))
    palette.setColor(QPalette.ColorRole.PlaceholderText, QColor("#000000"))
    palette.setColor(QPalette.ColorRole.Accent, QColor("#009faa"))

    return palette

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setPalette(customPalette())
    window=GroundStationApp()
    app.exec()