"""
Joystick manager — polls a connected joystick via pygame in a background thread
and exposes Qt signals for axes, button presses, and connection state changes.

Usage:
    from serial.joystick import JoystickManager
    joy = JoystickManager()
    joy.roll_changed.connect(my_slot)
    joy.is_connected()        # bool
    joy.stop()                # call on shutdown
"""

import os
import warnings

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
warnings.filterwarnings("ignore", message="pkg_resources is deprecated")

import pygame
from PyQt6.QtCore import QObject, QThread, pyqtSignal

_DEADZONE = 0.05


class _JoystickWorker(QThread):
    roll_changed      = pyqtSignal(float)
    pitch_changed     = pyqtSignal(float)
    yaw_changed       = pyqtSignal(float)
    button_pressed    = pyqtSignal(int)
    connected_changed = pyqtSignal(bool)

    def __init__(self, sensitivity: float = 0.05, update_interval_ms: int = 20):
        super().__init__() # todo check if defaults good
        self._running = True
        self._sensitivity = sensitivity
        self._update_interval_ms = update_interval_ms
        self._last_emitted_axis_values = {0: None, 1: None, 2: None}

    @staticmethod
    def _dead(v: float) -> float:
        return 0.0 if abs(v) < _DEADZONE else v

    def _try_connect(self):
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            self._last_emitted_axis_values = {0: None, 1: None, 2: None}
            self.connected_changed.emit(True)
            return joystick
        return None

    def _emit_axis_if_needed(self, axis: int, value: float):
        previous_value = self._last_emitted_axis_values.get(axis)
        if previous_value is None or abs(value - previous_value) > self._sensitivity:
            self._last_emitted_axis_values[axis] = value
            if axis == 0:
                self.roll_changed.emit(value)
            elif axis == 1:
                self.pitch_changed.emit(value)
            elif axis == 2:
                self.yaw_changed.emit(value)

    def run(self):
        pygame.init()
        pygame.joystick.init()
        pygame.display.init()

        if not pygame.display.get_init():
            return

        pygame.display.set_mode((1, 1))

        js = self._try_connect()

        while self._running:
            latest_axis_values = {}

            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    js = self._try_connect()
                elif event.type == pygame.JOYDEVICEREMOVED:
                    js = None
                    self._last_emitted_axis_values = {0: None, 1: None, 2: None}
                    self.connected_changed.emit(False)
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_pressed.emit(event.button)
                elif event.type == pygame.JOYAXISMOTION and js is not None:
                    latest_axis_values[event.axis] = self._dead(event.value)

            for axis, value in latest_axis_values.items():
                self._emit_axis_if_needed(axis, value)

            self.msleep(self._update_interval_ms)

        pygame.quit()

    def stop(self):
        self._running = False

    def set_sensitivity(self, sensitivity: float):
        self._sensitivity = sensitivity

    def set_update_interval_ms(self, update_interval_ms: int):
        self._update_interval_ms = update_interval_ms


class JoystickManager(QObject):
    """
    Public interface for joystick input.

    Signals
    -------
    roll_changed(float)       axis 0, deadzone-filtered, -1 … +1
    pitch_changed(float)      axis 1, deadzone-filtered, -1 … +1
    yaw_changed(float)        axis 2 (twist), deadzone-filtered, -1 … +1
    button_pressed(int)       button index, emitted only on press (not release)
    connected_changed(bool)   True when a joystick is detected / reconnected
    """

    roll_changed      = pyqtSignal(float)
    pitch_changed     = pyqtSignal(float)
    yaw_changed       = pyqtSignal(float)
    button_pressed    = pyqtSignal(int)
    connected_changed = pyqtSignal(bool)

    def __init__(self, parent=None, sensitivity: float = 0.05, update_interval_ms: int = 20):
        super().__init__(parent)
        self._connected = False

        self._worker = _JoystickWorker(sensitivity=sensitivity, update_interval_ms=update_interval_ms)
        self._worker.roll_changed.connect(self.roll_changed)
        self._worker.pitch_changed.connect(self.pitch_changed)
        self._worker.yaw_changed.connect(self.yaw_changed)
        self._worker.button_pressed.connect(self.button_pressed)
        self._worker.connected_changed.connect(self._on_connection_changed)
        self._worker.start()

    def _on_connection_changed(self, connected: bool):
        self._connected = connected
        self.connected_changed.emit(connected)

    def is_connected(self) -> bool:
        return self._connected

    def set_sensitivity(self, sensitivity: float):
        self._worker.set_sensitivity(sensitivity)

    def set_update_interval_ms(self, update_interval_ms: int):
        self._worker.set_update_interval_ms(update_interval_ms)

    def stop(self):
        self._worker.stop()
        self._worker.wait()
