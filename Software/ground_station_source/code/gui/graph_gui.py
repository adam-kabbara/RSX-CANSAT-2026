"""
Front end GUI elements for graph window
"""

from plotter.plotters import DynamicPlotter, DynamicPlotter_MultiLine
from . import cosmetics
import os
from PyQt6.QtCore import Qt, QUrl
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QLabel,
    QGridLayout,
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QSystemTrayIcon,
    QApplication, QListWidget, QFrame, QListWidgetItem, QAbstractItemView
)
from PyQt6.QtWebEngineWidgets import QWebEngineView
import webbrowser

class GraphWindow(QMainWindow):

    def __init__(self, parent=None):

        super().__init__(parent)

        self.graphs = []
        self.plotters = []
        self._packets_recv = 0
        self._packets_sent = 0
        self._graph_time_window = 500 # how long data stays on graph
        self._screen_width_cm = 32.1
        self._screen_height_cm = 20
        self.previous_state = "Unknown"

        self.setWindowTitle("Live Data")
        icon_path = os.path.join(os.path.dirname(__file__), '..', 'media', 'icon.png')
        self.setWindowIcon(QIcon(icon_path))
        tray = QSystemTrayIcon()
        tray.setIcon(QIcon(icon_path))
        tray.setVisible(True)
        tray.show()

        # CENTRAL WIDGET
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        pixel_width = int((self._screen_width_cm / 2.54) * 96)
        self.central_widget.setFixedWidth(pixel_width)
        pixel_height = int((self._screen_height_cm / 2.54) * 96)
        self.central_widget.setFixedHeight(pixel_height)

        graph_parent_group = QHBoxLayout(self.central_widget)
        
        grid_container = QWidget()
        graph_grid_layout = QGridLayout(grid_container)

        graph_info = [
            {"title": "Altitude", "lines": 1, "x_unit": "s", "y_unit": "m"},
            {"title": "Voltage", "lines": 1, "x_unit": "s", "y_unit": "V"},
            {"title": "Current", "lines": 1, "x_unit": "s", "y_unit": "mA"},
            {"title": "Gyro RPY", "lines": 3, "x_unit": "s", "y_unit": "deg/s"},
            {"title": "Accel RPY ", "lines": 3, "x_unit": "s", "y_unit": "deg/s^2"},
            {"title": "GPS Map", "lines": 0, "x_unit": "s", "y_unit": "m"}
        ]   
        
        self.graph_title_to_index = {
            "Altitude" : 0,
            "Voltage" : 1,
            "Current": 2,
            "Gyro" : 3,
            "Accel" : 4,
            "GPS" : 5
        }

        # Loop through each graph and create a plot using the plot classes
        for i, entry in enumerate(graph_info):

            if entry["lines"] == 1:
                plotter = DynamicPlotter(title=entry["title"], 
                                         timewindow=self._graph_time_window,
                                         x_unit=entry["x_unit"],
                                         y_unit=entry["y_unit"])
            elif entry["lines"] != 1:
                plotter = DynamicPlotter_MultiLine(title=entry["title"], 
                                                   timewindow=self._graph_time_window, 
                                                   num_lines=entry["lines"],
                                                   x_unit=entry["x_unit"],
                                                   y_unit=entry["y_unit"])
            if entry["lines"] != 0: 
                self.plotters.append(plotter)
                graph_grid_layout.addWidget(plotter.get_graph_object(), i // 2, i % 2)
        
            if entry["title"] == "GPS Map":
                map_widget = QWebEngineView()
                # For an HTTP address use a plain QUrl (fromLocalFile is for filesystem paths)
                map_widget.setUrl(QUrl("http://127.0.0.1:5000"))
                # Keep a reference to the widget for later updates (and to avoid GC)
                self.gps_map_webview = map_widget
                map_widget.setMinimumSize(480, 320)
                graph_grid_layout.addWidget(map_widget, i // 2, i % 2)
            

        graph_parent_group.addWidget(grid_container, stretch=8)
        
        # Sidebar to show all current graph values
        sidebar_widget = QWidget()
        sidebar = QVBoxLayout(sidebar_widget)
        
        credit_label = QLabel("Made by the Engineers of RSX at the University of Toronto")
        credit_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        credit_label.setFont(cosmetics.credit_font())
        credit_label.setWordWrap(True)

        live_graph_values = QFormLayout()

        self.sidebar_fields_data = [
            ("Port", "CLOSED"),
            ("Calibration", "Unknown"),
            ("Temperature", "0.0 °C"),
            ("Pressure", "0.0 kPa"),
            ("Mode", "Unknown"),
            ("Mission Time", "00:00:00"),
            ("Packets", "0/0"),
            ("Satellites", "0"),
            ("Camera 1", "Unknown"),
            ("Camera 2", "Unknown"),
            ("GPS Altitude", "0.0 m"),
            ("GPS Time", "00:00:00"),
            ("CMD ECHO", "N/A")
        ]

        self.state_label_index = {
            "IDLE": 0,
            "LAUNCH_PAD": 1,
            "ASCENT": 2,
            "APOGEE": 3,
            "RELEASE": 4,
            "DESCENT": 5,
            "PROBE_RELEASE": 6,
            "PAYLOAD_RELEASE": 7,
            "LANDED": 8
        }
        self.state_labels = ("IDLE", "LAUNCH_PAD", "ASCENT", "APOGEE", "RELEASE", "DESCENT", "PROBE_RELEASE", "PAYLOAD_RELEASE", "LANDED")
        self.state_labels_display = ("IDLE", "LAUNCH PAD", "ASCENT", "APOGEE", "RELEASE", "DESCENT", "PROBE REL",
                             "PAYLD REL", "LANDED")

        self.sidebar_data_labels = []

        self.sidebar_data_dict = {name: idx for idx, (name, _) in enumerate(self.sidebar_fields_data)}

        # Previous state list
        self.previous_list = QListWidget()
        self.previous_list.setStyleSheet("border-radius: 0px;")
        self.previous_list.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.previous_list.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.previous_list.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.previous_list.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.previous_list.setFixedHeight(80)
        self.previous_list.setStyleSheet("background-color: transparent; color:black; font-size: 10px; font-family: Roboto Mono;")

        # Next state list
        self.next_list = QListWidget()
        self.next_list.setStyleSheet("border-radius: 0px;")
        for item in self.state_labels_display:
            _pending_item = QListWidgetItem(item)
            cosmetics.set_next_states(_pending_item)
            self.next_list.addItem(_pending_item)
        self.next_list.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.next_list.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.next_list.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.next_list.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.next_list.setFixedHeight(80)
        self.next_list.setStyleSheet("background-color: transparent;")

        # Create state label separately
        self.state_label = QLabel()
        self.state_label.setFont(cosmetics.state_label_font())
        self.state_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.state_label.setFont(cosmetics.state_label_font())
        self._reset_states()

        # --- Create Title Widgets ---
        self.previous_label_title = QLabel("Previous")
        self.previous_label_title.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.previous_label_title.setFont(cosmetics.state_grid_title_font())
        self.next_label_title = QLabel("Next")
        self.next_label_title.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.next_label_title.setFont(cosmetics.state_grid_title_font())

        # Current state display
        self.current_state_display = QLabel("Unknown")
        self.current_state_display.setFrameShape(QFrame.Shape.Panel)
        self.current_state_display.setStyleSheet("color: blue; padding: 5px;")

        for field_name, field_value in self.sidebar_fields_data:
            # Handle state separately
            if field_name == "State":
                continue

            # Create the field label and data label
            field_label = QLabel(f"{field_name}:")
            data_label = QLabel(cosmetics.data_status_init_color(field_value))

            # Set fonts
            field_label.setFont(cosmetics.sidebar_field_font())
            data_label.setFont(cosmetics.sidebar_data_font())

            # Add them to your lists (or directly to your layout if needed)
            self.sidebar_data_labels.append(data_label)

            live_graph_values.addRow(field_label, data_label)

        self.set_port_text_closed()
        
        form_group = QGroupBox()
        form_group.setLayout(live_graph_values)

        state_visual_box = QGroupBox()
        state_visual_layout = QVBoxLayout(state_visual_box)
        state_grid_layout = QGridLayout()
        state_grid_layout.addWidget(self.state_label, 0, 0, 1, 0, Qt.AlignmentFlag.AlignLeft)

        state_grid_layout.addWidget(self.previous_label_title, 2, 0)
        state_grid_layout.addWidget(self.next_label_title, 2, 1)

        state_grid_layout.addWidget(self.previous_list, 3, 0)
        state_grid_layout.addWidget(self.next_list, 3, 1)

        state_grid_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)
        state_visual_layout.addLayout(state_grid_layout)

        sidebar.addWidget(form_group)
        sidebar.addWidget(state_visual_box)
        sidebar.addStretch()
        sidebar.addWidget(credit_label)

        graph_parent_group.addWidget(sidebar_widget, stretch=2)
        graph_parent_group.setSpacing(15)

    def set_port_text_closed(self):
        self.sidebar_data_labels[self.sidebar_data_dict.get("Port")].setText(cosmetics.data_status_red("CLOSED"))
        
    def set_port_text_open(self):
        self.sidebar_data_labels[self.sidebar_data_dict.get("Port")].setText(cosmetics.data_status_green("OPEN"))

    def reset_data(self):
        self._packets_recv = 0
        self._packets_sent = 0
        self.update_packet_label()
        self.update_state(None, reset=True)
        self._initiate_data_fields()
        for plotter in self.plotters:
            plotter.reset_plot()

    def _initiate_data_fields(self):
        self.set_port_text_closed()
        for name, val in self.sidebar_fields_data:
            if name == "Port":
                continue
            self.sidebar_data_labels[self.sidebar_data_dict.get(name)].setText(cosmetics.data_status_init_color(val))

    def update_packet_count(self):
        self._packets_recv += 1

    def update_packets_sent(self, count):
        self._packets_sent = count

    def get_packet_count(self):
        return self._packets_recv

    def update_cal_status(self, str):
        self.sidebar_data_labels[self.sidebar_data_dict.get("Calibration")].setText(cosmetics.data_status_blue(str))
    
    def update_temp(self, val):
        self.sidebar_data_labels[self.sidebar_data_dict.get("Temperature")].setText(cosmetics.data_status_blue(str(val)))

    def update_pressure(self, val):
        self.sidebar_data_labels[self.sidebar_data_dict.get("Pressure")].setText(cosmetics.data_status_blue(str(val)))

    # Modified state update function
    def update_state(self, state_str, reset=False):
        if reset:
            self._reset_states()
            return
        if state_str == "Unknown":
            self.previous_state = "Unknown"
            self.state_label.setText("Current " + cosmetics.data_status_init_color(state_str))
            return
        elif state_str != self.previous_state:
            if state_str in self.state_labels:
                current_state_index = self.state_label_index.get(state_str)
                previous_state_index = self.state_label_index.get(self.previous_state)
                if self.previous_state == "Unknown" or current_state_index - previous_state_index == 1:
                    # Populate a single stage
                    _pending_item = QListWidgetItem(self.state_labels_display[current_state_index])
                    cosmetics.set_previous_states(_pending_item)
                    self.previous_list.addItem(_pending_item)
                    self.previous_list.scrollToBottom()
                else:
                    for state in self.state_labels_display[previous_state_index + 1:current_state_index]:
                        # Populate skipped stages
                        _pending_item_skipped = QListWidgetItem(state)
                        cosmetics.set_skipped_states(_pending_item_skipped)
                        self.previous_list.addItem(_pending_item_skipped)
                    _pending_item = QListWidgetItem(self.state_labels_display[current_state_index])
                    cosmetics.set_previous_states(_pending_item)
                    self.previous_list.addItem(_pending_item)
                    self.previous_list.scrollToBottom()
                self.next_list.clear()
                for state in self.state_labels_display[current_state_index + 1:]:
                    _pending_item_next = QListWidgetItem(state)
                    cosmetics.set_next_states(_pending_item_next)
                    self.next_list.addItem(_pending_item_next)
                self.next_list.scrollToTop()
            self.previous_state = state_str
            self.state_label.setText("Current " + cosmetics.data_status_blue(state_str))

    def _reset_states(self):
        self.previous_state = "Unknown"
        self.state_label.setText("Current " + cosmetics.data_status_init_color("Unknown"))
        self.previous_list.clear()
        self.next_list.clear()
        for item in self.state_labels_display:
            _pending_item = QListWidgetItem(item)
            cosmetics.set_next_states(_pending_item)
            self.next_list.addItem(_pending_item)

    def update_mode(self, str):
        self.sidebar_data_labels[self.sidebar_data_dict.get("Mode")].setText(cosmetics.data_status_blue(str))

    def update_mission_time(self, str):
        self.sidebar_data_labels[self.sidebar_data_dict.get("Mission Time")].setText(cosmetics.data_status_blue(str))

    def update_packet_label(self):
        self.sidebar_data_labels[self.sidebar_data_dict.get("Packets")].setText(
            cosmetics.data_status_blue(f"{self._packets_recv}/{self._packets_sent}"))

    def update_sats(self, val):
        self.sidebar_data_labels[self.sidebar_data_dict.get("Satellites")].setText(cosmetics.data_status_blue(str(val)))
    
    def update_camera1_status(self, str):
        self.sidebar_data_labels[self.sidebar_data_dict.get("Camera 1")].setText(cosmetics.data_status_blue(str))

    def update_camera2_status(self, str):
        self.sidebar_data_labels[self.sidebar_data_dict.get("Camera 2")].setText(cosmetics.data_status_blue(str))

    def update_gps_alt(self, val):
        self.sidebar_data_labels[self.sidebar_data_dict.get("GPS Altitude")].setText(cosmetics.data_status_blue(str(val)))

    def update_gps_time(self, str):
        self.sidebar_data_labels[self.sidebar_data_dict.get("GPS Time")].setText(cosmetics.data_status_blue(str))
    
    def update_cmd_echo(self, str):
        self.sidebar_data_labels[self.sidebar_data_dict.get("CMD ECHO")].setText(cosmetics.data_status_blue(str))

    def update_alt_graph(self, data):
        self.plotters[self.graph_title_to_index.get("Altitude")].update_plot(data)
    
    def update_volt_graph(self, data):
        self.plotters[self.graph_title_to_index.get("Voltage")].update_plot(data)

    def update_current_graph(self, data):
        self.plotters[self.graph_title_to_index.get("Current")].update_plot(data)

    def update_gyro_graph(self, data):
        self.plotters[self.graph_title_to_index.get("Gyro")].update_plot(data)

    def update_accel_graph(self, data):
        self.plotters[self.graph_title_to_index.get("Accel")].update_plot(data)
    
    def update_map_view(self, data):
        self.plotters[self.graph_title_to_index.get("GPS")].update_plot(data)


    def closeEvent(self, event):
        app = QApplication.instance()
        app.quit()
        event.accept()