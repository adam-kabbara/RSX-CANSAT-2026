"""
Front end GUI elements for graph window
"""

from plotter.plotters import DynamicPlotter, DynamicPlotter_MultiLine
import gui.cosmetics
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QLabel,
    QGridLayout,
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
)

class GraphWindow(QMainWindow):

    def __init__(self, parent=None):

        self.graphs = []
        self.plotters = []

        super().__init__(parent)

        # CENTRAL WIDGET
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        graph_parent_group = QHBoxLayout(self.central_widget)
        
        grid_container = QWidget()
        graph_grid_layout = QGridLayout(grid_container)

        graph_info = [
            {"title": "Altitude", "lines": 1, "x_unit": "s", "y_unit": "m"},
            {"title": "Voltage", "lines": 1, "x_unit": "s", "y_unit": "V"},
            {"title": "Current", "lines": 1, "x_unit": "s", "y_unit": "mA"},
            {"title": "Gyro RPY", "lines": 3, "x_unit": "s", "y_unit": "deg/s"},
            {"title": "Accel RPY ", "lines": 3, "x_unit": "s", "y_unit": "deg/s^2"},
            {"title": "GPS Map", "lines": 1, "x_unit": "s", "y_unit": "m"}
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
                                         timewindow=self.__graph_time_window,
                                         x_unit=entry["x_unit"],
                                         y_unit=entry["y_unit"])
            else:
                plotter = DynamicPlotter_MultiLine(title=entry["title"], 
                                                   timewindow=self.__graph_time_window, 
                                                   num_lines=entry["lines"],
                                                   x_unit=entry["x_unit"],
                                                   y_unit=entry["y_unit"])

            self.plotters.append(plotter)
            graph_grid_layout.addWidget(plotter.get_graph_object(), i // 2, i % 2)

        graph_parent_group.addWidget(grid_container, stretch=8)
        
        # Sidebar to show all current graph values
        sidebar_widget = QWidget()
        sidebar = QVBoxLayout(sidebar_widget)

        self.info_label = QLabel("Live Values")
        self.info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.info_label.setFont(gui.cosmetics.sidebar_title_font())

        self.credit_label = QLabel("Made by the Engineers of RSX at the University of Toronto")
        self.credit_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.credit_label.setFont(gui.cosmetics.credit_font())

        self.live_graph_values = QFormLayout()

        # TODO: Color
        sidebar_fields_data = [
            ("Temperature", "0.0 °C"),
            ("Pressure", "0.0 kPa"),
            ("State", "Unknown"),
            ("Mode", "Unknown"),
            ("State", "Unknown"),
            ("Mission Time", "00:00:00"),
            ("Packets", "0/0"),
            ("Satellites", "0"),
            ("Camera 1", "Unknown"),
            ("Camera 2", "Unknown"),
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
            field_label.setFont(gui.cosmetics.sidebar_field_font())
            data_label.setFont(gui.cosmetics.sidebar_data_font())

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

        self.showMaximized()