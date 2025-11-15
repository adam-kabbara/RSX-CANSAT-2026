"""
Front end GUI elements for command window
"""

from enum import Enum
import os
from . import cosmetics
from serial.serial import SerialManager
from .graph_gui import GraphWindow
from data.process import DataProcessor
from command.commands import Commands
from PyQt6.QtGui import QColor, QIcon, QIntValidator, QTextCursor
from PyQt6.QtCore import Qt, QTime
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

class CommandButtonGroup(Enum):
    MAIN = 0
    MODE = 1
    ADVANCED = 2
    SENSORS = 3
    CONNECTION = 4
    TELEMETRY = 5

class CommandWindow(QMainWindow):

    def __init__(self, serial: SerialManager, graph_ui: GraphWindow, processor: DataProcessor, parent=None):

        super().__init__(parent)

        self.__set_time_id        = 1
        self.__camera_id          = 1
        self.__servo_id           = -1
        self.__servo_val          = -1
        self.__CURRENT_CMD_WINDOW = None
        self.__last_msg           = None
        self.__last_msg_sat       = False
        self.__last_prop_item     = None
        self.__log_repeat_count   = 0
        self._serial              = serial
        self._graph_ui            = graph_ui
        self._processor           = processor
        self.command_manager      = Commands(self._serial)

        self._serial.error_catch.connect(self.update_gui_log_error)
        self._serial.fatal_catch.connect(self.serial_fatal)
        self._serial.print_catch.connect(self.update_gui_log)

        self.command_manager.print_signal.connect(self.update_gui_log)

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

        self.button_connect = QPushButton("OPEN PORT")
        self.button_connect.setFont(button_font)
        self.button_connect.clicked.connect(self.open_port)
        self.button_connect.hide()

        self.button_connect_close = QPushButton("CLOSE PORT")
        self.button_connect_close.setFont(button_font)
        self.button_connect_close.clicked.connect(self.close_port)
        self.button_connect_close.hide()

        self.button_telemetry = QPushButton("TELEMETRY")
        self.button_telemetry.setFont(button_font)
        self.button_telemetry.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.TELEMETRY))

        self.button_transmit_on = QPushButton("START MISSION")
        self.button_transmit_on.setFont(button_font)
        self.button_transmit_on.clicked.connect(self.start_transmission)
        self.button_transmit_on.hide()

        # TODO: close csv file and any other clean up on receiving confirmation of mission end
        self.button_transmit_off = QPushButton("END MISSION")
        self.button_transmit_off.setFont(button_font)
        self.button_transmit_off.clicked.connect(self.command_manager.command__end_mission)
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

        self.button_sim_mode_enable = QPushButton("SIM MODE ENABLE")
        self.button_sim_mode_enable.setFont(button_font)
        self.button_sim_mode_enable.clicked.connect(lambda: self.command_manager.command__sim_mode("ENABLE"))
        self.button_sim_mode_enable.hide()

        self.button_sim_mode_activate = QPushButton("SIM MODE ACTIVATE")
        self.button_sim_mode_activate.setFont(button_font)
        self.button_sim_mode_activate.clicked.connect(lambda: self.command_manager.command__sim_mode("ACTIVATE"))
        self.button_sim_mode_activate.hide()

        self.button_sim_mode_disable = QPushButton("SIM MODE DISABLE")
        self.button_sim_mode_disable.setFont(button_font)
        self.button_sim_mode_disable.clicked.connect(lambda: self.command_manager.command__sim_mode("DISABLE"))
        self.button_sim_mode_disable.hide()

        self.button_refresh_ports = QPushButton("REFRESH PORTS")
        self.button_refresh_ports.setFont(button_font)
        self.button_refresh_ports.clicked.connect(lambda: self.refresh_ports(True))
        self.button_refresh_ports.hide()

        self.button_get_log_data = QPushButton("GET CANSAT LOG DATA")
        self.button_get_log_data.setFont(button_font)
        self.button_get_log_data.clicked.connect(self.command_manager.command__get_log)
        self.button_get_log_data.hide()

        self.button_sensor_control = QPushButton("SENSOR CONTROL")
        self.button_sensor_control.setFont(button_font)
        self.button_sensor_control.clicked.connect(lambda: self.command_group_change_buttons(CommandButtonGroup.SENSORS))

        self.button_altitude_cal = QPushButton("CALIBRATE ALTITUDE")
        self.button_altitude_cal.setFont(button_font)
        self.button_altitude_cal.clicked.connect(self.command_manager.command__alt_cal)
        self.button_altitude_cal.hide()

        self.button_test_connection = QPushButton("CHECK CONNECTION")
        self.button_test_connection.setFont(button_font)
        self.button_test_connection.clicked.connect(self.command_manager.command__check_connection)
        self.button_test_connection.hide()

        ### Program servo
        program_servo_box = QHBoxLayout()

        self.servo_id_field = QComboBox()
        self.servo_id_field.setPlaceholderText("SELECT SERVO")
        self.servo_id_field.addItem("Servo 1", 0)
        self.servo_id_field.addItem("Servo 2", 1)
        self.servo_id_field.addItem("Servo 3", 2)
        self.servo_id_field.addItem("Servo 4", 3)
        self.servo_id_field.setFont(button_font)
        self.servo_id_field.activated.connect(self.servo_id_edited)

        self.servo_val_field = QLineEdit()
        self.servo_val_field.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.servo_val_field.setMaxLength(3)
        self.servo_val_field.setStyleSheet(cosmetics.servo_val_stylesheet())
        int_validator = QIntValidator(self)
        self.servo_val_field.setValidator(int_validator)
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
        self.camera_id_field.setPlaceholderText("SELECT CAMERA")
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

        self.probe_release_force = QPushButton("FORCE PROBE RELEASE")
        self.probe_release_force.setFont(button_font)
        self.probe_release_force.clicked.connect(self.command_manager.command__probe_release)
        self.probe_release_force.hide()

        self.camera_status_button = QPushButton("GET CAMERA STATUS")
        self.camera_status_button.setFont(button_font)
        self.camera_status_button.clicked.connect(self.command_manager.command__cam_status)
        self.camera_status_button.hide()

        self.team_id_field = QLineEdit()
        self.team_id_field.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.team_id_field.setMaxLength(9)
        self.team_id_field.setStyleSheet(cosmetics.team_id_stylesheet())
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
        commands_layout.addWidget(self.button_connect_close)
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
            self.button_connect_close,
            self.combo_select_port,
            self.button_refresh_ports,
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
        self.update_logs(msg, sat_msg=False, color=cosmetics.gui_log_fatal_color())
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
        time = QTime.currentTime().toString('h:mm AP').replace(' ', '\u00A0')
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

    def team_id_edited(self):
        self.team_id_field.clearFocus()
        self.__TEAM_ID = int(self.team_id_field.text())
        self.update_gui_log(f"Updated local TEAM ID to '{self.__TEAM_ID}'")
    
    def servo_id_edited(self, index):
        self.__servo_id = self.servo_id_field.itemData(index)
    
    def servo_val_edited(self):
        self.servo_val_field.clearFocus()
        self.__servo_val = int(self.servo_val_field.text())

    def camera_id_edited(self, index):
        self.__camera_id = self.camera_id_field.itemText(index)
    
    def set_time_field_edited(self, index):
        self.__set_time_id = self.set_time_field.itemData(index)
        
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
            self._processor.reset_csv()
            self.__last_msg = None
            self.__last_msg_sat = False
            self.__log_repeat_count = 0

    def closeEvent(self, event):
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
