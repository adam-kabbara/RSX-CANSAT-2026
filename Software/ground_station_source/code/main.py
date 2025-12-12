"""
Package runner
"""
from PyQt6.QtWidgets import QApplication
from PyQt6.QtWidgets import QMainWindow
from serial.serial import SerialManager
from gui.command_gui import CommandWindow
from gui.graph_gui import GraphWindow
from data.process import DataProcessor
import gui.cosmetics
import sys
import argparse

parser = argparse.ArgumentParser()
group = parser.add_mutually_exclusive_group()
group.add_argument('-g', action='store_true')
group.add_argument('-c', action='store_true')

def center_window(window: QMainWindow):

    window.adjustSize()
    
    screen = window.screen()
    geo = window.frameGeometry()

    geo.moveCenter(screen.availableGeometry().center())

    window.move(geo.topLeft())

if __name__ == "__main__":

    args = parser.parse_args()

    app = QApplication(sys.argv)

    serial = SerialManager()
    graphing = GraphWindow()
    processor = DataProcessor(graphing)
    command = CommandWindow(serial, graphing, processor)

    app.lastWindowClosed.connect(app.quit)

    serial.recv_data_signal.connect(processor.process_data)

    gui.cosmetics.set_app_style(app)

    center_window(graphing)
    center_window(command)

    if(args.g):
        graphing.show()
    elif(args.c):
        command.show()
    else:
        graphing.show()
        command.show()

    app.exec()