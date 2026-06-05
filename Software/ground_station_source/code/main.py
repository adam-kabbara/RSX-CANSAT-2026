"""
Package runner
** Use python main.py -g to only show graphing window
** Use python main.py -c to only show command window
** Use python main.py --omit-screen-resolution to test without configured display placement
"""
#PLACE ON TOP CUZ OTHER IMPROTS MAY TAKE SO LONG, THIS GIVE IMMEDIATE FEEDBACK TO USER
from data.audio_tts import AudioTTS
tts = AudioTTS()
tts.speak("Ground station software booting up")

from PyQt6.QtCore import QPoint, QSize, Qt, QTimer
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
parser.add_argument(
    '--omit-screen-resolution',
    action='store_true',
    help='Show normal centered windows without configured resolution, fullscreen, or borderless placement.',
)

def center_window(window: QMainWindow):

    window.adjustSize()

    screen = window.screen()
    geo = window.frameGeometry()

    geo.moveCenter(screen.availableGeometry().center())

    window.move(geo.topLeft())


def _screen_size(screen):
    geometry = screen.geometry()
    return QSize(geometry.width(), geometry.height())


def _screen_match_score(screen, target_size: QSize):
    screen_size = _screen_size(screen)
    return abs(screen_size.width() - target_size.width()) + abs(screen_size.height() - target_size.height())


def _select_screen(target_size: QSize, used_screen_names: set[str]):
    screens = QApplication.screens()
    if not screens:
        return QApplication.primaryScreen()

    unused_screens = [screen for screen in screens if screen.name() not in used_screen_names]
    candidate_screens = unused_screens or screens

    exact_matches = [
        screen for screen in candidate_screens
        if _screen_size(screen) == target_size
    ]
    if exact_matches:
        return exact_matches[0]

    return min(candidate_screens, key=lambda screen: _screen_match_score(screen, target_size))


def _move_window_to_screen(window: QMainWindow, screen):
    if screen is None:
        return window.screen().geometry()

    window.winId()
    window_handle = window.windowHandle()
    if window_handle is not None:
        window_handle.setScreen(screen)

    screen_geometry = screen.geometry()
    window.move(screen_geometry.topLeft())
    return screen_geometry


def show_configured_window(window: QMainWindow, window_name: str, used_screen_names: set[str]):
    settings = gui.cosmetics.window_settings(window_name)
    target_width, target_height = settings["resolution"]
    target_size = QSize(target_width, target_height)
    screen = _select_screen(target_size, used_screen_names)

    if screen is not None:
        used_screen_names.add(screen.name())
    screen_geometry = _move_window_to_screen(window, screen)

    display_mode = settings["display_mode"]
    if display_mode == "fullscreen":
        window.setGeometry(screen_geometry)
        window.showFullScreen()
        return

    width = min(target_size.width(), screen_geometry.width())
    height = min(target_size.height(), screen_geometry.height())
    window.setWindowFlag(Qt.WindowType.FramelessWindowHint, True)
    window.setFixedSize(width, height)
    window.move(screen_geometry.topLeft() if screen is not None else QPoint(0, 0))
    window.show()


def show_testing_window(window: QMainWindow):
    center_window(window)
    window.show()


def show_main_window(window: QMainWindow, window_name: str, used_screen_names: set[str], omit_screen_resolution: bool):
    if omit_screen_resolution:
        show_testing_window(window)
        return

    show_configured_window(window, window_name, used_screen_names)

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

    if visualization is not None:
        center_window(visualization)

    used_screen_names = set()
    if(args.g):
        show_main_window(graphing, "live_data", used_screen_names, args.omit_screen_resolution)
    elif(args.c):
        show_main_window(command, "command_panel", used_screen_names, args.omit_screen_resolution)
    else:
        show_main_window(graphing, "live_data", used_screen_names, args.omit_screen_resolution)
        show_main_window(command, "command_panel", used_screen_names, args.omit_screen_resolution)

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
