"""
Front end GUI elements for command window
"""

import os
from enum import Enum

from PyQt6.QtCore import Qt, QTimer, QDateTime
from PyQt6.QtGui import QColor, QIcon, QIntValidator, QDoubleValidator
from PyQt6.QtWidgets import (
    QMainWindow,
    QPushButton,
    QWidget,
    QLabel,
    QGridLayout,
    QGroupBox,
    QLineEdit,
    QVBoxLayout,
    QHBoxLayout,
    QComboBox,
    QSystemTrayIcon,
    QMessageBox,
    QApplication, QAbstractItemView, QTableWidget, QTableWidgetItem, QHeaderView
)

from command.commands import Commands
from data.process import DataProcessor
from data.simp import SimpManager
from serial.joystick import JoystickManager
from serial.serial import SerialManager
from . import cosmetics
from .graph_gui import GraphWindow


class CommandButtonGroup(Enum):
    MAIN = 0
    ACTUATION = 1
    ADVANCED = 2
    SENSORS = 3
    CONNECTION = 4
    MISSION_CONTROL = 5
    SETUP = 6

class CommandWindow(QMainWindow):

    def __init__(self, serial: SerialManager, graph_ui: GraphWindow, processor: DataProcessor, simp: SimpManager, parent=None):

        super().__init__(parent)

        self.__set_time_id        = 0 # 0 for computer time, 1 for GPS time
        self.__camera_id          = 1
        self.__mec_id             = 1
        self.__servo_id           = 0 # first option is nosecone by gui default
        self.__servo_val          = -1
        self.__dc_motor_val       = 0
        self.__joy_sensitivity    = 0.05
        self.__joy_update_ms      = 20
        self.__gps_lat            = 0.0
        self.__gps_lon            = 0.0
        self.__gps_rad            = 0.0
        self.__sim_mode           = "ENABLE"
        self.__flight_ctrl        = "AUTONOMOUS"
        self.__custom_msg         = ""
        
        self.__CURRENT_CMD_WINDOW = None
        self.__last_msg           = None
        self.__last_msg_sat       = False
        self.__last_prop_item     = None
        self.__log_repeat_count   = 0
        self._serial              = serial
        self._graph_ui            = graph_ui
        self._processor           = processor
        self._simp                = simp

        self._joystick            = JoystickManager()
        self.command_manager      = Commands(self._serial, self._joystick, self._graph_ui)

        # Joystick axis accumulator for attitude indicator debug feed
        self._joy_roll  = 0.0
        self._joy_pitch = 0.0
        self._joy_yaw   = 0.0

        self._serial.error_catch.connect(self.update_gui_log_error)
        self._serial.fatal_catch.connect(self.serial_fatal)
        self._serial.print_catch.connect(self.update_gui_log)

        self.command_manager.print_signal.connect(self.update_gui_log)

        self._joystick.connected_changed.connect(self._graph_ui.update_joystick_status)
        self._joystick.button_pressed.connect(self.command_manager._on_joystick_button)
        self._joystick.roll_changed.connect(self.command_manager._on_joy_roll)
        self._joystick.pitch_changed.connect(self.command_manager._on_joy_pitch)
        self._joystick.yaw_changed.connect(self.command_manager._on_joy_yaw)
        self._graph_ui.update_joystick_status(False)

        self._processor.log_end_signal.connect(self.logfile_finish)
        self._processor.log_begin_signal.connect(self.logfile_start)
        self._processor.file_error_signal.connect(self.update_gui_log_error)
        self._processor.sat_resp_signal.connect(self.update_cansat_log)
        self._processor.sat_error_signal.connect(self.update_cansat_log_error)

        self.setWindowTitle("Command Center")
        icon_path = os.path.join(os.path.dirname(__file__), '..', 'media', 'icon.png')
        self.setWindowIcon(QIcon(icon_path))
        tray = QSystemTrayIcon()
        tray.setIcon(QIcon(icon_path))
        tray.setVisible(True)
        tray.show()

        # ------ FONTS ------ #
        button_font = cosmetics.button_font()
        log_font = cosmetics.log_font()
        # ------ FONTS ------ #

        # CENTRAL WIDGET
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        grid_layout = QGridLayout(self.central_widget)
        grid_layout.setHorizontalSpacing(10)
        grid_layout.setVerticalSpacing(20)
        grid_layout.setContentsMargins(10, 10, 10, 10)

        # ------ COMMANDS GROUP ------ #
        commands_group_box = QGroupBox()
        commands_group_box.setMinimumHeight(300)
        commands_group_box.setMinimumWidth(500)
        commands_layout = QVBoxLayout(commands_group_box)

        self.button_actuation = QPushButton("ACTUATION")
        self.button_actuation.setFont(button_font)
        self.button_actuation.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.ACTUATION))

        self.button_connection_group = QPushButton("CONNECTION")
        self.button_connection_group.setFont(button_font)
        self.button_connection_group.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.CONNECTION))

        self.button_connect = QPushButton("OPEN PORT")
        self.button_connect.setFont(button_font)
        self.button_connect.clicked.connect(self.open_port)
        self.button_connect.hide()

        self.button_connect_close = QPushButton("CLOSE PORT")
        self.button_connect_close.setFont(button_font)
        self.button_connect_close.clicked.connect(self.close_port)
        self.button_connect_close.hide()

        self.button_telemetry = QPushButton("MISSION CONTROL")
        self.button_telemetry.setFont(button_font)
        self.button_telemetry.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.MISSION_CONTROL))

        self.button_transmit_on = QPushButton("START MISSION")
        self.button_transmit_on.setFont(button_font)
        self.button_transmit_on.clicked.connect(self.start_transmission)
        self.button_transmit_on.hide()

        self.button_transmit_off = QPushButton("STOP MISSION")
        self.button_transmit_off.setFont(button_font)
        self.button_transmit_off.clicked.connect(self.end_transmission)
        self.button_transmit_off.hide()

        self.button_advanced = QPushButton("ADVANCED")
        self.button_advanced.setFont(button_font)
        self.button_advanced.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.ADVANCED))

        self.button_setup = QPushButton("SETUP")
        self.button_setup.setFont(button_font)
        self.button_setup.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.SETUP))

        self.button_back = QPushButton("BACK")
        self.button_back.setFont(button_font)
        self.button_back.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.MAIN))
        self.button_back.hide()

        self.combo_select_port = QComboBox()
        self.combo_select_port.setFont(button_font)
        self.combo_select_port.activated.connect(self.port_selected)
        self.combo_select_port.hide()

        self.button_restart = QPushButton("RESTART PROCESSOR")
        self.button_restart.setFont(button_font)
        self.button_restart.clicked.connect(self.command_manager.command__restart)
        self.button_restart.hide()

        set_time_box = QHBoxLayout()

        self.button_set_time = QPushButton("SET TIME")
        self.button_set_time.setFont(button_font)
        self.button_set_time.clicked.connect(lambda: self.command_manager.command__send_time(self.__set_time_id))
        self.button_set_time.hide()

        self.set_time_field = QComboBox()
        self.set_time_field.addItem("COMPUTER", 0)
        self.set_time_field.addItem("GPS", 1)
        self.set_time_field.setFont(button_font)
        self.set_time_field.activated.connect(self.set_time_field_edited)
        self.set_time_field.hide()

        set_time_box.addWidget(self.button_set_time)
        set_time_box.addWidget(self.set_time_field)

        self.button_reset_mission = QPushButton("RESET ALL DATA")
        self.button_reset_mission.setFont(button_font)
        self.button_reset_mission.clicked.connect(self.reset_mission)
        self.button_reset_mission.hide()

        sim_mode_box = QHBoxLayout()

        ### Set GPS
        set_gps_box = QHBoxLayout()
        self.button_set_gps = QPushButton("SET GPS COOR")
        self.button_set_gps.setFont(button_font)

        self.gps_lat = QLineEdit()
        self.gps_lat.setPlaceholderText("Latitude")
        self.gps_lat.setValidator(QDoubleValidator())
        self.gps_lat.setStyleSheet(cosmetics.team_id_stylesheet())
        self.gps_lat.editingFinished.connect(self.gps_lat_edited)
        

        self.gps_lon = QLineEdit()
        self.gps_lon.setPlaceholderText("Longitude")
        self.gps_lon.setValidator(QDoubleValidator())
        self.gps_lon.setStyleSheet(cosmetics.team_id_stylesheet())
        self.gps_lon.editingFinished.connect(self.gps_lon_edited)
        

        self.gps_rad = QLineEdit()
        self.gps_rad.setPlaceholderText("Radius")
        self.gps_rad.setValidator(QDoubleValidator())
        self.gps_rad.setStyleSheet(cosmetics.team_id_stylesheet())
        self.gps_rad.editingFinished.connect(self.gps_rad_edited)

        self.button_set_gps.clicked.connect(self.set_gps_coordinates)
        self.button_set_gps.hide()

        self.gps_lat.hide()
        self.gps_lon.hide()
        self.gps_rad.hide()

        set_gps_box.addWidget(self.button_set_gps)
        set_gps_box.addWidget(self.gps_lat)
        set_gps_box.addWidget(self.gps_lon)
        set_gps_box.addWidget(self.gps_rad)

        self.button_set_sim_mode = QPushButton("SET SIM MODE")
        self.button_set_sim_mode.setFont(button_font)
        self.button_set_sim_mode.clicked.connect(lambda: self.command_manager.command__sim_mode(self.__sim_mode))
        self.button_set_sim_mode.hide()

        self.sim_mode_field = QComboBox()
        self.sim_mode_field.addItem("ENABLE")
        self.sim_mode_field.addItem("ACTIVATE")
        self.sim_mode_field.addItem("DISABLE")
        self.sim_mode_field.setFont(button_font)
        self.sim_mode_field.activated.connect(self.sim_mode_field_edited)
        self.sim_mode_field.hide()

        sim_mode_box.addWidget(self.button_set_sim_mode)
        sim_mode_box.addWidget(self.sim_mode_field)

        flight_ctrl_box = QHBoxLayout()

        self.button_set_flight_ctrl = QPushButton("SET CONTROL MODE")
        self.button_set_flight_ctrl.setFont(button_font)
        self.button_set_flight_ctrl.clicked.connect(lambda: self.command_manager.command__flight_ctrl(self.__flight_ctrl))
        self.button_set_flight_ctrl.hide()

        self.flight_ctrl_field = QComboBox()
        self.flight_ctrl_field.addItem("AUTONOMOUS")
        self.flight_ctrl_field.addItem("MANUAL")
        self.flight_ctrl_field.setFont(button_font)
        self.flight_ctrl_field.activated.connect(self.flight_ctrl_field_edited)
        self.flight_ctrl_field.hide()

        flight_ctrl_box.addWidget(self.button_set_flight_ctrl)
        flight_ctrl_box.addWidget(self.flight_ctrl_field)

        self.button_refresh_ports = QPushButton("REFRESH PORTS")
        self.button_refresh_ports.setFont(button_font)
        self.button_refresh_ports.clicked.connect(lambda: self.refresh_ports(True))
        self.button_refresh_ports.hide()

        self.button_get_log_data = QPushButton("GET CANSAT LOG DATA")
        self.button_get_log_data.setFont(button_font)
        self.button_get_log_data.clicked.connect(self.command_manager.command__get_log)
        self.button_get_log_data.hide()

        self.button_sensor_control = QPushButton("CALIBRATION")
        self.button_sensor_control.setFont(button_font)
        self.button_sensor_control.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.SENSORS))

        self.button_altitude_cal = QPushButton("CALIBRATE ALTITUDE")
        self.button_altitude_cal.setFont(button_font)
        self.button_altitude_cal.clicked.connect(self.command_manager.command__alt_cal)
        self.button_altitude_cal.hide()

        self.button_accel_cal = QPushButton("CALIBRATE ACCELERATION")
        self.button_accel_cal.setFont(button_font)
        self.button_accel_cal.clicked.connect(lambda: self.command_manager.command__cal("ACCE"))
        self.button_accel_cal.hide()

        self.button_tof_cal = QPushButton("CALIBRATE ToF")
        self.button_tof_cal.setFont(button_font)
        self.button_tof_cal.clicked.connect(lambda: self.command_manager.command__cal("TOF"))
        self.button_tof_cal.hide()

        self.button_mag_cal = QPushButton("CALIBRATE MAGNETIC")
        self.button_mag_cal.setFont(button_font)
        self.button_mag_cal.clicked.connect(lambda: self.command_manager.command__cal("MAG"))
        self.button_mag_cal.hide()

        self.button_gyro_cal = QPushButton("CALIBRATE GYRO")
        self.button_gyro_cal.setFont(button_font)
        self.button_gyro_cal.clicked.connect(lambda: self.command_manager.command__cal("GYRO"))
        self.button_gyro_cal.hide()

        self.button_next_cal_step = QPushButton("NEXT CALIBRATION STEP")
        self.button_next_cal_step.setFont(button_font)
        self.button_next_cal_step.clicked.connect(lambda: self.command_manager.command__cal("NEXT"))
        self.button_next_cal_step.hide()

        self.button_test_connection = QPushButton("CHECK CONNECTION")
        self.button_test_connection.setFont(button_font)
        self.button_test_connection.clicked.connect(self.command_manager.command__check_connection)
        self.button_test_connection.hide()

        joystick_sensitivity_box = QHBoxLayout()

        self.button_set_joystick_sensitivity = QPushButton("SET JOYSTICK SENSITIVITY")
        self.button_set_joystick_sensitivity.setFont(button_font)

        self.joystick_sensitivity_field = QLineEdit()
        self.joystick_sensitivity_field.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.joystick_sensitivity_field.setStyleSheet(cosmetics.servo_val_stylesheet())
        self.joystick_sensitivity_field.setValidator(QDoubleValidator(self))
        self.joystick_sensitivity_field.setText(str(self.__joy_sensitivity))
        self.joystick_sensitivity_field.setPlaceholderText(f"sensitivity cur={self.__joy_sensitivity}")
        self.joystick_sensitivity_field.editingFinished.connect(self.joystick_sensitivity_edited)
        self.button_set_joystick_sensitivity.clicked.connect(lambda: self.command_manager._set_joystick_sensitivity(self.__joy_sensitivity))

        self.button_set_joystick_sensitivity.hide()
        self.joystick_sensitivity_field.hide()

        joystick_sensitivity_box.addWidget(self.button_set_joystick_sensitivity)
        joystick_sensitivity_box.addWidget(self.joystick_sensitivity_field)

        joystick_update_box = QHBoxLayout()

        self.button_set_joystick_update_interval = QPushButton("SET JOYSTICK UPDATE")
        self.button_set_joystick_update_interval.setFont(button_font)

        self.joystick_update_interval_field = QLineEdit()
        self.joystick_update_interval_field.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.joystick_update_interval_field.setStyleSheet(cosmetics.servo_val_stylesheet())
        self.joystick_update_interval_field.setValidator(QIntValidator(self))
        self.joystick_update_interval_field.setText(str(self.__joy_update_ms))
        self.joystick_update_interval_field.editingFinished.connect(self.joystick_update_interval_edited)
        self.button_set_joystick_update_interval.clicked.connect(lambda: self.command_manager._set_joystick_update_interval(self.__joy_update_ms))

        self.button_set_joystick_update_interval.hide()
        self.joystick_update_interval_field.hide()

        joystick_update_box.addWidget(self.button_set_joystick_update_interval)
        joystick_update_box.addWidget(self.joystick_update_interval_field)

        ### Program servo
        set_dc_motor_box = QHBoxLayout()

        self.set_dc_motor_button = QPushButton("SET DC MOTOR")
        self.set_dc_motor_button.setFont(button_font)
        
        self.dc_motor_val_field = QLineEdit()
        self.dc_motor_val_field.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.dc_motor_val_field.setStyleSheet(cosmetics.servo_val_stylesheet())
        self.dc_motor_val_field.setValidator(QIntValidator(self))
        self.dc_motor_val_field.setPlaceholderText("Time motor on is ms ; neg=stow ; pos=deploy")
        self.dc_motor_val_field.editingFinished.connect(self.dc_motor_val_edited)
        self.set_dc_motor_button.clicked.connect(lambda: self.command_manager.command__set_dc(self.__dc_motor_val))

        self.set_dc_motor_button.hide()

        self.dc_motor_val_field.hide()

        set_dc_motor_box.addWidget(self.set_dc_motor_button)
        set_dc_motor_box.addWidget(self.dc_motor_val_field)

        program_servo_box = QHBoxLayout()

        self.servo_id_field = QComboBox()
        self.servo_id_field.addItem("Nosecone", 0)
        self.servo_id_field.addItem("Container", 1)
        self.servo_id_field.addItem("Elevator", 2)
        self.servo_id_field.addItem("Aileron", 3)
        self.servo_id_field.addItem("Egg", 4)
        self.servo_id_field.setFont(button_font)
        self.servo_id_field.setPlaceholderText("Select servo")
        self.servo_id_field.setCurrentIndex(-1)
        self.servo_id_field.activated.connect(self.servo_id_edited)

        self.servo_val_field = QLineEdit()
        self.servo_val_field.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.servo_val_field.setStyleSheet(cosmetics.servo_val_stylesheet())
        self.servo_val_field.setValidator(QIntValidator(self))
        self.servo_val_field.setPlaceholderText("Angle (-90 to 90)")
        self.servo_val_field.editingFinished.connect(self.servo_val_edited)

        self.program_servo_button = QPushButton(" PROGRAM SERVO ")
        self.program_servo_button.setFont(button_font)
        self.program_servo_button.clicked.connect(lambda: self.command_manager.command__write_servo(self.__servo_id, self.__servo_val))
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
        self.camera_id_field.addItem("CAMERA1")
        self.camera_id_field.addItem("CAMERA2")
        self.camera_id_field.setFont(button_font)
        self.camera_id_field.activated.connect(self.camera_id_edited)

        self.program_camera_button = QPushButton("TOGGLE CAMERA")
        self.program_camera_button.setFont(button_font)
        self.program_camera_button.clicked.connect(lambda: self.command_manager.command__toggle_camera(self.__camera_id))

        program_camera_box.addWidget(self.program_camera_button)
        program_camera_box.addWidget(self.camera_id_field)
        
        self.camera_id_field.hide()
        self.program_camera_button.hide()
        ### end program camera

        force_release_box = QHBoxLayout()
        self.mec_release_field = QComboBox()
        self.mec_release_field.addItem("NOSECONE RELEASE")
        self.mec_release_field.addItem("CPL RELEASE")
        self.mec_release_field.addItem("WING DEPLOYMENT")
        self.mec_release_field.addItem("EGG RELEASE")
        self.mec_release_field.setFont(button_font)
        self.mec_release_field.activated.connect(self.mec_rel_edited)

        self.mec_activate_button = QPushButton("ACTIVATE")
        self.mec_activate_button.setFont(button_font)
        self.mec_activate_button.clicked.connect(lambda: self.command_manager.command__mec_release(self.__mec_id))

        self.mec_release_field.hide()
        self.mec_activate_button.hide()

        force_release_box.addWidget(self.mec_release_field)
        force_release_box.addWidget(self.mec_activate_button)

        self.camera_status_button = QPushButton("GET CAMERA STATUS")
        self.camera_status_button.setFont(button_font)
        self.camera_status_button.clicked.connect(self.command_manager.command__cam_status)
        self.camera_status_button.hide()

        self.custom_msg_field = QLineEdit()
        self.custom_msg_field.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.custom_msg_field.setMaxLength(50)
        self.custom_msg_field.setStyleSheet(cosmetics.team_id_stylesheet())
        self.custom_msg_field.editingFinished.connect(self.custom_msg_edited)
        self.button_send_custom = QPushButton("SEND CUSTOM MSG")
        self.button_send_custom.setFont(button_font)
        self.button_send_custom.clicked.connect(lambda: self.command_manager.command__custom(self.__custom_msg))
        
        custom_msg_editing_box = QHBoxLayout()
        custom_msg_editing_box.addWidget(self.button_send_custom)
        custom_msg_editing_box.addWidget(self.custom_msg_field)
        self.button_send_custom.hide()
        self.custom_msg_field.hide()
        
        # MAIN buttons
        commands_layout.addWidget(self.button_connection_group)
        commands_layout.addWidget(self.button_telemetry)
        commands_layout.addWidget(self.button_setup)
        commands_layout.addWidget(self.button_sensor_control)
        commands_layout.addWidget(self.button_actuation)
        commands_layout.addWidget(self.button_advanced)
        
        # CONNECTION buttons
        commands_layout.addWidget(self.combo_select_port)
        commands_layout.addWidget(self.button_connect)
        commands_layout.addWidget(self.button_connect_close)
        commands_layout.addWidget(self.button_refresh_ports)
        commands_layout.addWidget(self.button_test_connection)
        
        # MISSION_CONTROL buttons
        commands_layout.addWidget(self.button_transmit_on)
        commands_layout.addWidget(self.button_transmit_off)
        commands_layout.addWidget(self.button_get_log_data)
        
        # ADVANCED buttons
        commands_layout.addWidget(self.button_restart)
        commands_layout.addWidget(self.button_reset_mission)
        commands_layout.addLayout(joystick_sensitivity_box)
        commands_layout.addLayout(joystick_update_box)
        commands_layout.addLayout(custom_msg_editing_box)
        
        # SENSORS buttons
        commands_layout.addWidget(self.button_altitude_cal)
        commands_layout.addWidget(self.button_accel_cal)
        commands_layout.addWidget(self.button_tof_cal)
        commands_layout.addWidget(self.button_mag_cal)
        commands_layout.addWidget(self.button_gyro_cal)
        commands_layout.addWidget(self.button_next_cal_step)
        
        # SETUP buttons
        commands_layout.addLayout(set_time_box)
        commands_layout.addLayout(sim_mode_box)
        commands_layout.addLayout(flight_ctrl_box)
        commands_layout.addLayout(set_gps_box)
        
        # ACTUATION buttons
        commands_layout.addLayout(set_dc_motor_box)
        commands_layout.addLayout(program_servo_box)
        commands_layout.addLayout(program_camera_box)
        commands_layout.addWidget(self.camera_status_button)
        commands_layout.addLayout(force_release_box)
        
        # BACK button (universal)
        commands_layout.addWidget(self.button_back)

        grid_layout.setColumnStretch(0,1)

        grid_layout.addWidget(commands_group_box, 0, 0)

        # Store buttons in groups so we can control them later
        self.buttons_main = [
            self.button_advanced,
            self.button_setup,
            self.button_connection_group,
            self.button_actuation,
            self.button_sensor_control,
            self.button_telemetry
        ]
        
        self.buttons_adv = [
            self.button_reset_mission,
            self.button_restart,
            self.button_set_joystick_sensitivity,
            self.joystick_sensitivity_field,
            self.button_set_joystick_update_interval,
            self.joystick_update_interval_field,
            self.button_back,
            self.custom_msg_field,
            self.button_send_custom
        ]

        self.buttons_mission_control = [
            self.button_transmit_on,
            self.button_transmit_off,
            self.button_get_log_data,
            self.button_back
        ]

        self.buttons_actuation = [
            self.set_dc_motor_button,
            self.dc_motor_val_field,
            self.program_servo_button,
            self.servo_id_field,
            self.servo_val_field,
            self.program_camera_button,
            self.camera_id_field,
            self.camera_status_button,
            self.mec_release_field,
            self.mec_activate_button,
            self.button_back
        ]

        self.buttons_setup = [
            self.button_set_gps,
            self.gps_lat,
            self.gps_lon,
            self.gps_rad,
            self.button_back,
            self.button_set_time,
            self.set_time_field,
            self.button_set_sim_mode,
            self.sim_mode_field,
            self.button_set_flight_ctrl,
            self.flight_ctrl_field
        ]

        self.buttons_sensor = [
            self.button_back,
            self.button_altitude_cal,
            self.button_accel_cal,
            self.button_tof_cal,
            self.button_mag_cal,
            self.button_gyro_cal,
            self.button_next_cal_step
        ]

        self.buttons_connection = [
            self.button_test_connection,
            self.button_back,
            self.button_connect,
            self.button_connect_close,
            self.combo_select_port,
            self.button_refresh_ports
        ]

        self.get_log_overlay = QLabel("Logfile collection in progress.", self)
        self.get_log_overlay.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.get_log_overlay.setStyleSheet(cosmetics.log_overlay_stylesheet())
        self.get_log_overlay.setGeometry(self.rect())
        self.get_log_overlay.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
        self.get_log_overlay.hide()
        # ------ END COMMANDS GROUP ------ #

        # ------  LOG GROUP ------ #
        gui_log_widget = QWidget()
        gui_log_layout = QVBoxLayout(gui_log_widget)
        sat_log_widget = QWidget()
        sat_log_layout = QVBoxLayout(sat_log_widget)
        gui_log_widget.setFixedHeight(300)
        gui_log_widget.setFixedWidth(500)
        sat_log_widget.setFixedHeight(300)
        sat_log_widget.setFixedWidth(500)

        gui_log_title = QLabel("Command Log")
        gui_log_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        gui_log_title.setFont(log_font)

        cansat_log_title = QLabel("CanSat Log")
        cansat_log_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cansat_log_title.setFont(log_font)

        self.gui_log = QTableWidget()
        self.gui_log.setColumnCount(2)
        self.gui_log.setWordWrap(True)
        self.gui_log.verticalHeader().setVisible(False)
        self.gui_log.horizontalHeader().setVisible(False)
        self.gui_log.setShowGrid(False)
        self.gui_log.setHorizontalHeaderLabels(["Prop", "Message"])
        self.gui_log.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.gui_log.setTextElideMode(Qt.TextElideMode.ElideNone)
        self.gui_log.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.gui_log.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.gui_log.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.gui_log.setStyleSheet(cosmetics.log_stylesheet())

        self.cansat_log = QTableWidget()
        self.cansat_log.setColumnCount(2)
        self.cansat_log.setWordWrap(True)
        self.cansat_log.verticalHeader().setVisible(False)
        self.cansat_log.horizontalHeader().setVisible(False)
        self.cansat_log.setShowGrid(False)
        self.cansat_log.setHorizontalHeaderLabels(["Prop", "Message"])
        self.cansat_log.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.cansat_log.setTextElideMode(Qt.TextElideMode.ElideNone)
        self.cansat_log.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.cansat_log.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.cansat_log.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.cansat_log.setStyleSheet(cosmetics.log_stylesheet())

        gui_log_layout.addWidget(gui_log_title)
        gui_log_layout.addWidget(self.gui_log)

        sat_log_layout.addWidget(cansat_log_title)
        sat_log_layout.addWidget(self.cansat_log)

        grid_layout.setColumnStretch(1,1)

        grid_layout.addWidget(gui_log_widget, 0, 1)

        grid_layout.setColumnStretch(2,1)

        grid_layout.addWidget(sat_log_widget, 0, 2)
        # ------ END LOG GROUP ------ #

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.get_log_overlay.setGeometry(self.rect())

    def serial_fatal(self, msg):
        self.update_logs(msg, sat_msg=False, color=cosmetics.gui_log_error_color())
        self._graph_ui.set_port_text_closed()

    def update_gui_log(self, msg):
        self.update_logs(msg, sat_msg = False, color=cosmetics.gui_log_normal_color())

    def update_gui_log_error(self, msg):
        self.update_logs(msg, sat_msg = False, color=cosmetics.gui_log_error_color())

    def update_cansat_log(self, msg):
        self.update_logs(msg, sat_msg = True, color=cosmetics.sat_log_normal_color())

    def update_cansat_log_error(self, msg):
        self.update_logs(msg, sat_msg = True, color=cosmetics.sat_log_error_color())

    def update_logs(self, msg, sat_msg = False, color="black"):
        time = QDateTime.currentDateTimeUtc().toString('h:mm AP').replace(' ', '\u00A0')
        msg_item = QTableWidgetItem(f"{msg}")
        msg_item.setForeground(QColor(color))

        if sat_msg == False:
            target_log = self.gui_log
        else:
            target_log = self.cansat_log

        # Check repeat msgs
        if msg == self.__last_msg and sat_msg == self.__last_msg_sat:
            # Update prop column
            self.__log_repeat_count = self.__log_repeat_count + 1
            prop_item = self.__last_prop_item
            prop_item.setText(f"{time}\n[{self.__log_repeat_count}]")
            target_log.resizeRowsToContents()
        else:
            # Insert a new row
            if self.__log_repeat_count > 1:
                prop_item = QTableWidgetItem(f"{time}\n")
            else:
                prop_item = QTableWidgetItem(f"{time}")
            msg_item.setTextAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
            prop_item.setTextAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
            row = target_log.rowCount()
            target_log.insertRow(row)
            target_log.setItem(row, 0, prop_item)
            target_log.setItem(row, 1, msg_item)
            target_log.resizeRowsToContents()
            target_log.scrollToBottom()
            self.__log_repeat_count = 1
            self.__last_msg = msg
            self.__last_msg_sat = sat_msg
            self.__last_prop_item = prop_item

    # Change what buttons are shown in the commands box
    def command_group_change_buttons(self, mode):
        if mode == CommandButtonGroup.MISSION_CONTROL:
            self.control_buttons(self.buttons_main, hide=True)
            self.control_buttons(self.buttons_mission_control)
            self.__CURRENT_CMD_WINDOW = CommandButtonGroup.MISSION_CONTROL

        elif mode == CommandButtonGroup.MAIN:
            match self.__CURRENT_CMD_WINDOW:
                case CommandButtonGroup.ADVANCED:
                    self.control_buttons(self.buttons_adv, hide=True)
                case CommandButtonGroup.ACTUATION:
                    self.control_buttons(self.buttons_actuation, hide=True)
                case CommandButtonGroup.CONNECTION:
                    self.control_buttons(self.buttons_connection, hide=True)
                case CommandButtonGroup.SENSORS:
                    self.control_buttons(self.buttons_sensor, hide=True)
                case CommandButtonGroup.SETUP:
                    self.control_buttons(self.buttons_setup, hide=True)
                case CommandButtonGroup.MISSION_CONTROL:
                    self.control_buttons(self.buttons_mission_control, hide=True)
            self.control_buttons(self.buttons_main)
        
        elif mode == CommandButtonGroup.ADVANCED:
            self.control_buttons(self.buttons_main, hide=True)
            self.control_buttons(self.buttons_adv)
            self.__CURRENT_CMD_WINDOW = CommandButtonGroup.ADVANCED

        elif mode == CommandButtonGroup.ACTUATION:
            self.control_buttons(self.buttons_main, hide=True)
            self.control_buttons(self.buttons_actuation)
            self.__CURRENT_CMD_WINDOW = CommandButtonGroup.ACTUATION
        
        elif mode == CommandButtonGroup.SETUP:
            self.control_buttons(self.buttons_main, hide=True)
            self.control_buttons(self.buttons_setup)
            self.__CURRENT_CMD_WINDOW = CommandButtonGroup.SETUP

        elif mode == CommandButtonGroup.SENSORS:
            self.control_buttons(self.buttons_main, hide=True)
            self.control_buttons(self.buttons_sensor)
            self.__CURRENT_CMD_WINDOW = CommandButtonGroup.SENSORS
        
        elif mode == CommandButtonGroup.CONNECTION:
            self.combo_select_port.clear()
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

        ports_info = self._serial.search_ports()

        if len(ports_info) == 0:
            self.combo_select_port.addItem("No available ports")
        else:        
            for name, desc in ports_info:
                port_txt = name + ": " + desc
                self.combo_select_port.addItem(port_txt)

        if(b_print == True):
            self.update_gui_log("Attempted port refresh")

    def port_selected(self):
        self._serial.set_port(self.combo_select_port.currentIndex())

    def open_port(self):
        if self._serial.open_port():
            self._graph_ui.set_port_text_open()
    
    def close_port(self):
        if self._serial.close_port():
            self._graph_ui.set_port_text_closed()

    def start_transmission(self):
        if self.command_manager.command__start_mission():
            self._graph_ui.reset_data()
    
    def end_transmission(self):
        if self.command_manager.command__end_mission():
            if self._simp.simp_check():
                self._simp.simp_disable()
            self._processor.close_csv()
            self.update_gui_log("Telemetry CSV saved.")

    def custom_msg_edited(self):
        self.custom_msg_field.clearFocus()
        self.__custom_msg = self.custom_msg_field.text()
        self.update_gui_log(f"Custom message set to: '{self.__custom_msg}'")
    
    def dc_motor_val_edited(self):
        self.dc_motor_val_field.clearFocus()
        if self.dc_motor_val_field.text():
            self.__dc_motor_val = int(self.dc_motor_val_field.text())

    def joystick_sensitivity_edited(self):
        self.joystick_sensitivity_field.clearFocus()
        if self.joystick_sensitivity_field.text():
            self.__joy_sensitivity = float(self.joystick_sensitivity_field.text())

    def joystick_update_interval_edited(self):
        self.joystick_update_interval_field.clearFocus()
        if self.joystick_update_interval_field.text():
            self.__joy_update_ms = int(self.joystick_update_interval_field.text())

    def gps_lat_edited(self):
        self.gps_lat.clearFocus()
        if self.gps_lat.text():
            self.__gps_lat = float(self.gps_lat.text())

    def gps_lon_edited(self):
        self.gps_lon.clearFocus()
        if self.gps_lon.text():
            self.__gps_lon = float(self.gps_lon.text())

    def gps_rad_edited(self):
        self.gps_rad.clearFocus()
        if self.gps_rad.text():
            self.__gps_rad = float(self.gps_rad.text())

    def set_gps_coordinates(self):
        lat_text = self.gps_lat.text().strip()
        lon_text = self.gps_lon.text().strip()
        rad_text = self.gps_rad.text().strip()

        if not lat_text or not lon_text or not rad_text:
            self.update_gui_log_error("ERROR: Enter latitude, longitude, and radius before setting GPS coordinates.")
            return

        try:
            lat = float(lat_text)
            lon = float(lon_text)
            radius = float(rad_text)
        except ValueError:
            self.update_gui_log_error("ERROR: GPS coordinates and radius must be numeric.")
            return

        if not (-90 <= lat <= 90):
            self.update_gui_log_error("ERROR: Latitude must be between -90 and 90.")
            return

        if not (-180 <= lon <= 180):
            self.update_gui_log_error("ERROR: Longitude must be between -180 and 180.")
            return

        if radius <= 0:
            self.update_gui_log_error("ERROR: Radius must be greater than 0.")
            return

        self.__gps_lat = lat
        self.__gps_lon = lon
        self.__gps_rad = radius

        if self.command_manager.command__set_gps(lat, lon, radius):
            self._graph_ui.draw_gps_circle(lat, lon, radius)

    def servo_id_edited(self, index):
        self.__servo_id = self.servo_id_field.itemData(index)
    
    def servo_val_edited(self):
        self.servo_val_field.clearFocus()
        self.__servo_val = int(self.servo_val_field.text())

    def camera_id_edited(self, index):
        self.__camera_id = self.camera_id_field.itemText(index)

    def mec_rel_edited(self, index):
        self.__mec_id = self.mec_release_field.itemText(index)
    
    def set_time_field_edited(self, index):
        self.__set_time_id = self.set_time_field.itemData(index)
    
    def sim_mode_field_edited(self, index):
        self.__sim_mode = self.sim_mode_field.itemText(index)

    def flight_ctrl_field_edited(self, index):
        self.__flight_ctrl = self.flight_ctrl_field.itemText(index)
        
    def reset_mission(self):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setWindowTitle("CONFIRM: RESET ALL DATA")
        msg_box.setText("Are you sure you reset all mission data?")
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        msg_box.setDefaultButton(QMessageBox.StandardButton.No)

        response = msg_box.exec()
        if response == QMessageBox.StandardButton.Yes:
            self.gui_log.clear()
            self.cansat_log.clear()
            self._graph_ui.reset_data()
            if not self._processor.reset_csv():
                self.update_gui_log_error("ERROR: CSV WAS NOT ABLE TO RESET! DETAILS:")
                self.update_gui_log_error(self._processor.get_csv_error_msg())
            self.__last_msg = None
            self.__last_msg_sat = False
            self.__log_repeat_count = 0

    def run_payload_sim(self):
        self.refresh_ports(False)
        
        # Find SIM port index
        sim_index = -1
        for i in range(self.combo_select_port.count()):
            if "SIM" in self.combo_select_port.itemText(i):
                sim_index = i
                break
        
        if sim_index != -1:
            self.combo_select_port.setCurrentIndex(sim_index)
            self.port_selected()
            self.open_port()

            QTimer.singleShot(500, self._debug_calibrate)
        else:
            self.update_gui_log_error("DEBUG: SIM port not found")

    def _debug_calibrate(self):
        self.command_manager.command__alt_cal()
        QTimer.singleShot(500, self._debug_start_mission)

    def _debug_start_mission(self):
        self.start_transmission()

    def closeEvent(self, event):
        self._joystick.stop()
        self._processor.close_csv()
        self._processor.close_logfile()
        self._serial.close_port()
        app = QApplication.instance()
        app.quit()
        event.accept()

    def logfile_finish(self):
        self.get_log_overlay.hide()
        self.update_gui_log("Log data download complete.")

    def logfile_start(self):
        self.get_log_overlay.show()
