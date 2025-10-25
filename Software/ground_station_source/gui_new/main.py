"""
Package runner
"""
from PyQt6.QtWidgets import QApplication
from serial.serial import SerialManager
from gui.command_gui import CommandWindow
from gui.graph_gui import GraphWindow
from data.process import DataProcessor
import gui.cosmetics
import sys

if __name__ == "__main__":

    app = QApplication(sys.argv)

    serial = SerialManager()
    graphing = GraphWindow()
    processor = DataProcessor(graphing)
    command = CommandWindow(serial, graphing, processor)

    app.lastWindowClosed.connect(app.quit)

    serial.recv_data_signal.connect(processor.process_data)

    gui.cosmetics.set_app_style(app)

    graphing.show()
    command.show()

    app.exec()