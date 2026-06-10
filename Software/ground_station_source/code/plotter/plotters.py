"""
Classes for real-time plots
"""

import time
from collections import deque
import pyqtgraph as pg
from pyqtgraph import mkPen
from PyQt6.QtCore import Qt
import numpy as np
import gui.cosmetics

# Base graph plotting system
# Initialize plots and set fonts/colors
class BaseDynamicPlotter:
    
    def __init__(self, title, timewindow, x_unit, y_unit):
        self.timewindow = timewindow
        self.last_time = None
        self.base_line_color_idx = 0

        font = gui.cosmetics.graph_font()

        self.plt = pg.PlotWidget()
        self.plt.setBackground(gui.cosmetics.graph_background())
        self.plt.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.plt.setTitle(gui.cosmetics.graph_title(title))
        self.plt.showGrid(x=True, y=True)
        self.plt.getAxis('bottom').setStyle(tickFont=font)
        self.plt.getAxis('bottom').setLabel(gui.cosmetics.graph_axis(x_unit))
        self.plt.getAxis('left').setStyle(tickFont=font)
        self.plt.getAxis('left').setLabel(gui.cosmetics.graph_axis(y_unit))
    
    def get_pen(self, index):
        return mkPen(color=gui.cosmetics.graph_pen_color(index), width=gui.cosmetics.graph_pen_size())

    def reset_plot(self):
        raise NotImplementedError

    def update_plot(self, *args):
        raise NotImplementedError

    def get_graph_object(self):
        return self.plt

# Plotting system for regular graphs with 1 line
class DynamicPlotter(BaseDynamicPlotter):

    def __init__(self, title, timewindow, x_unit, y_unit):
        super().__init__(title, timewindow, x_unit, y_unit)
        self.databuffer = deque([0.0] * timewindow, maxlen=timewindow)
        self.x = np.linspace(-timewindow, 0, timewindow)
        self.y = np.zeros(self.databuffer.maxlen, dtype=float)
        self.curve = self.plt.plot(self.x, self.y, pen=self.get_pen(0))
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
class DynamicPlotterMultiLine(BaseDynamicPlotter):
    def __init__(self, title, timewindow, num_lines, x_unit, y_unit):
        super().__init__(title, timewindow, x_unit, y_unit)
        self.num_lines = num_lines
        self.databuffer = [deque([0.0] * timewindow, maxlen=timewindow) for _ in range(num_lines)]
        self.x = np.linspace(-timewindow, 0, timewindow)
        self.y = np.zeros(shape=(self.num_lines, timewindow), dtype=float)
        self.plt.getViewBox().setLimits(xMin=-5, xMax=5000, minXRange=5, yMin=-10000, yMax=10000, minYRange=2)
        self.curve = [
            self.plt.plot(self.x, self.y[i], pen=self.get_pen(self.base_line_color_idx + i))
            for i in range(self.num_lines)
        ]

        label_names = ["R/X", "P/Y", "Y/Z"]
        self.labels = []

        for i in range(min(self.num_lines, 3)):
            pen = self.get_pen(self.base_line_color_idx + i)
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