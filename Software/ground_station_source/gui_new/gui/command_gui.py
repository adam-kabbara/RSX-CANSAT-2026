"""
Front end GUI elements for command window
"""

from PyQt6.QtWidgets import QMainWindow
import cosmetics
from serial.serial import SerialManager, SerialPortToggleStatus
from enum import Enum
from PyQt6.QtGui import QColor, QIcon, QIntValidator, QTextCursor
from PyQt6.QtCore import Qt, QTime
from datetime import datetime, timezone
import time
from PyQt6.QtWidgets import (
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
    QTabWidget,
    QTextEdit,
)

class CommandButtonGroup(Enum):
    MAIN = 0
    MODE = 1
    ADVANCED = 2
    SENSORS = 3
    CONNECTION = 4
    TELEMETRY = 5

class CommandWindow(QMainWindow):

    def __init__(self, parent=None):

        self.__TEAM_ID            = 3114
        self.__set_time_id        = 1
        self.__CURRENT_CMD_WINDOW = None
        self._port_selected_idx   = None

        super().__init__(parent)

        self.setWindowTitle("Command Center")
        self.setWindowIcon(QIcon('../media/icon.png'))

        # ------ FONTS ------ #
        button_font = cosmetics.button_font()
        graph_sidebar_font = cosmetics.graph_sidebar_font()
        credit_font = cosmetics.credit_font()
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
        self.servo_val_field.setStyleSheet(cosmetics.servo_val_stylesheet())
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

        self.gui_simulation_button = QPushButton("Start GUI Simulation")
        self.gui_simulation_button.setFont(button_font)
        self.gui_simulation_button.clicked.connect(self.start_stop_gui_simulation) #TODO: WRITE THIS FUNCTION
        self.gui_simulation_button.hide()

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
        commands_layout.addWidget(self.button_show_map)
        commands_layout.addWidget(self.button_sim_mode_enable)
        commands_layout.addWidget(self.button_sim_mode_activate)
        commands_layout.addWidget(self.button_sim_mode_disable)
        commands_layout.addWidget(self.button_get_log_data)
        commands_layout.addWidget(self.probe_release_force)
        commands_layout.addLayout(team_id_editing_box)
        commands_layout.addWidget(self.gui_simulation_button)
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
            self.gui_simulation_button,
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

        gui_log_title = QLabel("Command Log")
        gui_log_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        gui_log_title.setFont(graph_sidebar_font)

        cansat_log_title = QLabel("CanSat Log")
        cansat_log_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cansat_log_title.setFont(graph_sidebar_font)

        self.gui_log = QTextEdit()
        self.gui_log.setReadOnly(True)
        self.gui_log.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.gui_log.setLineWrapMode(QTextEdit.LineWrapMode.WidgetWidth)
        self.gui_log.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.gui_log.setStyleSheet(cosmetics.log_stylesheet())

        self.cansat_log = QTextEdit()
        self.cansat_log.setReadOnly(True)
        self.cansat_log.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.cansat_log.setLineWrapMode(QTextEdit.LineWrapMode.WidgetWidth)
        self.cansat_log.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
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

        # ------ GRAPH GROUP ------ #
        graph_parent_group = QHBoxLayout()
        self.tab_widget = QTabWidget()
        graph_parent_group.addWidget(self.tab_widget, stretch=8)

        self.tab_widget.setStyleSheet(cosmetics.tab_widget_stylesheet())
        self.tab_widget.currentChanged.connect(self.on_tab_changed)

        self.showMaximized()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.get_log_overlay.setGeometry(self.rect())

    def update_gui_log(self, msg):
        self.update_logs(msg, sat_msg = False, color="black")

    def update_gui_log_error(self, msg):
        self.update_logs(msg, sat_msg = False, color="red")

    def update_cansat_log(self, msg):
        self.update_logs(msg, sat_msg = True, color="blue")

    def update_cansat_log_error(self, msg):
        self.update_logs(msg, sat_msg = True, color="red")

    def update_logs(self, msg, sat_msg = False, color="black"):
        if sat_msg == False:
            target_log = self.gui_log
        else:
            target_log = self.cansat_log

        current_time = QTime.currentTime().toString('h:mm AP')
        target_log.setTextColor(QColor(color))

        if msg == self.__last_msg and sat_msg == self.__last_msg_sat:
            self.__log_repeat_count += 1
            cursor = target_log.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            cursor.movePosition(QTextCursor.MoveOperation.StartOfBlock, QTextCursor.MoveMode.MoveAnchor)
            cursor.movePosition(QTextCursor.MoveOperation.EndOfBlock, QTextCursor.MoveMode.KeepAnchor)
            cursor.insertText(f"{current_time} [{self.__log_repeat_count}] {msg}")
            cursor.clearSelection()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            target_log.setTextCursor(cursor)
        else:
            self.__log_repeat_count = 1
            self.__last_msg = msg
            self.__last_msg_sat = sat_msg
            target_log.append(f"{current_time}      {msg}")

        scrollbar = target_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

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
        self._port_selected_idx = None

        ports_info = SerialManager.search_ports()

        if len(ports_info) == 0:
            self.combo_select_port.addItem("No available ports")
        else:        
            for name, desc in ports_info:
                port_txt = name + ": " + desc
                self.combo_select_port.addItem(port_txt)

        if(b_print == True):
            self.update_gui_log("Attempted port refresh")

    def port_selected(self):
        SerialManager.set_port(self.combo_select_port.currentIndex())

    def open_port(self):
        if SerialManager.open_port():
            self.set_port_text_open()
    
    def close_port(self):
        if SerialManager.close_port():
            self.set_port_text_closed()

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
            self.update_gui_log_error("ERROR: Enter a servo # and value first!")
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
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setWindowTitle("CONFIRM: REQUEST TRANSMISSION OF MISSION LOGFILE")
        msg_box.setText("THIS WILL BLOCK ALL OTHER PROCESSES UNTIL COMPLETE!!")
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        msg_box.setDefaultButton(QMessageBox.StandardButton.No)

        response = msg_box.exec()
        if response == QMessageBox.StandardButton.Yes:
            if(self.send_data("CMD,%d,GTLOGS,X" % self.__TEAM_ID)):
                self.update_gui_log("Attempting to retreive log data...")
        
    def toggle_transmission(self, toggle):
        # TODO
        '''
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
        '''

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

    def send_data(self, msg):
        SerialManager.send_data(msg)
        
    def reset_mission(self):     
        self.gui_log.clear()
        self.cansat_log.clear()
        #TODO

    def set_port_text_closed(self):
         self.label_port.setText(f'<span style="color:black;">Ground Port: \
                                              </span><span style="color:RED;">CLOSED</span>')
        
    def set_port_text_open(self):
        open_msg = "OPEN ON: " + self.__PORT_SELECTED_INFO.portName() + self.__PORT_SELECTED_INFO.description()
        self.label_port.setText(f'<span style="color:black;">Ground Port: \
                                              </span><span style="color:GREEN;">{open_msg}</span>')