"""
Package runner
** Use python main.py -g to only show graphing window
** Use python main.py -c to only show command window
"""
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication
from PyQt6.QtWidgets import QMainWindow
from serial.serial import SerialManager
from gui.command_gui import CommandWindow
from gui.graph_gui import GraphWindow
from gui.payload_visualization import PayloadVisualizationWindow
from data.process import DataProcessor
from data.simp import SimpManager
import gui.cosmetics
import sys
import argparse

parser = argparse.ArgumentParser()
group = parser.add_mutually_exclusive_group()
group.add_argument('-g', action='store_true')
group.add_argument('-c', action='store_true')
parser.add_argument('-psim', action='store_true')
parser.add_argument('-debug', action='store_true')
parser.add_argument('-vis', '--vis', action='store_true')

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
    visualization = PayloadVisualizationWindow() if args.vis else None
    simp = SimpManager(serial)
    processor = DataProcessor(graphing, simp)
    command = CommandWindow(serial, graphing, processor, simp)

    app.lastWindowClosed.connect(app.quit)

    serial.recv_data_signal.connect(processor.process_data)
    if visualization is not None:
        processor.telemetry_data_signal.connect(visualization.update_telemetry)

    gui.cosmetics.set_app_style(app)

    center_window(graphing)
    center_window(command)
    if visualization is not None:
        center_window(visualization)

    if(args.g):
        graphing.show()
    elif(args.c):
        command.show()
    else:
        graphing.show()
        command.show()

    if visualization is not None:
        visualization.show()

    if(args.psim):
        # Wait 500ms for GUI to load, then start the debug sequence
        QTimer.singleShot(500, command.run_payload_sim)

    if processor.csv_check() is False:
        command.update_gui_log_error("ERROR: CSV WAS NOT ABLE TO OPEN! DETAILS:")
        command.update_gui_log_error(processor.get_csv_error_msg())
        command.update_gui_log_error("NO DATA WILL BE SAVED IN THIS SESSION!")

    if simp.simp_check() is False:
        command.update_gui_log_error(f"ERROR: SIM DATA FILE WAS NOT ABLE TO OPEN! DETAILS:")
        command.update_gui_log_error(simp.get_error_msg())
        command.update_gui_log_error("SIMULATION MODE CANNOT BE USED IN THIS SESSION!")

    app.exec()
