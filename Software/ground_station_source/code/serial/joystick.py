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

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame
from PyQt6.QtCore import QObject, QThread, pyqtSignal

_DEADZONE = 0.05


class _JoystickWorker(QThread):
    roll_changed      = pyqtSignal(float)
    pitch_changed     = pyqtSignal(float)
    yaw_changed       = pyqtSignal(float)
    button_pressed    = pyqtSignal(int)
    connected_changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self._running = True

    @staticmethod
    def _dead(v: float) -> float:
        return 0.0 if abs(v) < _DEADZONE else v

    def run(self):
        pygame.init()
        pygame.joystick.init()
        pygame.display.init()

        if not pygame.display.get_init():
            return

        pygame.display.set_mode((1, 1))

        js = None

        def try_connect() -> bool:
            nonlocal js
            if pygame.joystick.get_count() > 0:
                js = pygame.joystick.Joystick(0)
                js.init()
                self.connected_changed.emit(True)
                return True
            return False

        try_connect()

        while self._running:
            pygame.event.pump()

            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    try_connect()
                elif event.type == pygame.JOYDEVICEREMOVED:
                    js = None
                    self.connected_changed.emit(False)
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_pressed.emit(event.button)

            if js is not None:
                try:
                    self.roll_changed.emit(self._dead(js.get_axis(0)))
                    self.pitch_changed.emit(self._dead(js.get_axis(1)))
                    yaw = self._dead(js.get_axis(2)) if js.get_numaxes() > 2 else 0.0
                    self.yaw_changed.emit(yaw)
                except Exception:
                    js = None
                    self.connected_changed.emit(False)

            self.msleep(20)

        pygame.quit()

    def stop(self):
        self._running = False


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

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False

        self._worker = _JoystickWorker()
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

    def stop(self):
        self._worker.stop()
        self._worker.wait()
