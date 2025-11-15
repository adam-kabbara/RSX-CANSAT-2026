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
    QApplication
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
            ("State", "Unknown"),
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

        self.sidebar_data_labels = []

        self.sidebar_data_dict = {name: idx for idx, (name, _) in enumerate(self.sidebar_fields_data)}

        for field_name, field_value in self.sidebar_fields_data:
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

        sidebar.addWidget(form_group)
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
        for idx, (_, value) in enumerate(self.sidebar_fields_data):
            self.sidebar_data_labels[idx].setText(value)
        for plotter in self.plotters:
            plotter.reset_plot()
    
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

    # TODO: State graph
    def update_state(self, str):
        self.sidebar_data_labels[self.sidebar_data_dict.get("State")].setText(cosmetics.data_status_blue(str))

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