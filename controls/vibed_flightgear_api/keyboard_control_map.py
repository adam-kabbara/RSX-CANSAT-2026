import time
import threading
from dataclasses import dataclass

from pynput import keyboard
from flightgear_python.fg_if import PropsConnection

# -----------------------------
# Config
# -----------------------------
FG_HOST = "localhost"
FG_PORT = 5500  # Must match --telnet=...,localhost,5500,tcp

HZ = 50.0
KEY_STEP = 0.08
DECAY_PER_SEC = 2.0
THROTTLE_STEP = 0.05

# FlightGear property paths
PROP_AILERON  = "/controls/flight/aileron"
PROP_ELEVATOR = "/controls/flight/elevator"
PROP_RUDDER   = "/controls/flight/rudder"
PROP_THROTTLE = "/controls/engines/engine[0]/throttle"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def approach(x, target, max_delta):
    if x < target:
        return min(x + max_delta, target)
    return max(x - max_delta, target)


@dataclass
class Controls:
    aileron: float = 0.0
    elevator: float = 0.0
    rudder: float = 0.0
    throttle: float = 0.2


class Bridge:
    def __init__(self):
        self.ctrl = Controls()
        self.key_state = set()
        self.running = True
        self.lock = threading.Lock()

        self.props = PropsConnection(FG_HOST, FG_PORT)

    # ---------- Keyboard ----------
    def on_key_press(self, key):
        with self.lock:
            self.key_state.add(key)

        if key == keyboard.Key.esc:
            print("Exiting...")
            self.running = False

    def on_key_release(self, key):
        with self.lock:
            self.key_state.discard(key)

    # ---------- Main loop ----------
    def loop(self):
        self.props.connect()

        dt = 1.0 / HZ
        last = time.time()

        print("Controls:")
        print("  W/S -> pitch")
        print("  A/D -> roll")
        print("  ←/→ -> rudder")
        print("  ↑/↓ -> throttle")
        print("  ESC -> quit")

        while self.running:
            now = time.time()
            elapsed = now - last
            last = now

            with self.lock:
                ks = set(self.key_state)

                # WASD
                if keyboard.KeyCode.from_char('a') in ks:
                    self.ctrl.aileron -= KEY_STEP
                if keyboard.KeyCode.from_char('d') in ks:
                    self.ctrl.aileron += KEY_STEP
                if keyboard.KeyCode.from_char('w') in ks:
                    self.ctrl.elevator -= KEY_STEP
                if keyboard.KeyCode.from_char('s') in ks:
                    self.ctrl.elevator += KEY_STEP

                # Arrow keys
                if keyboard.Key.left in ks:
                    self.ctrl.rudder -= KEY_STEP
                if keyboard.Key.right in ks:
                    self.ctrl.rudder += KEY_STEP
                if keyboard.Key.up in ks:
                    self.ctrl.throttle += THROTTLE_STEP
                if keyboard.Key.down in ks:
                    self.ctrl.throttle -= THROTTLE_STEP

                # Clamp ranges
                self.ctrl.aileron = clamp(self.ctrl.aileron, -1, 1)
                self.ctrl.elevator = clamp(self.ctrl.elevator, -1, 1)
                self.ctrl.rudder = clamp(self.ctrl.rudder, -1, 1)
                self.ctrl.throttle = clamp(self.ctrl.throttle, 0, 1)

                # Auto-center control surfaces
                max_delta = DECAY_PER_SEC * elapsed
                self.ctrl.aileron = approach(self.ctrl.aileron, 0, max_delta)
                self.ctrl.elevator = approach(self.ctrl.elevator, 0, max_delta)
                self.ctrl.rudder = approach(self.ctrl.rudder, 0, max_delta)

                a, e, r, t = (
                    self.ctrl.aileron,
                    self.ctrl.elevator,
                    self.ctrl.rudder,
                    self.ctrl.throttle,
                )

            # Send to FlightGear
            self.props.set_prop(PROP_AILERON, a)
            self.props.set_prop(PROP_ELEVATOR, e)
            self.props.set_prop(PROP_RUDDER, r)
            self.props.set_prop(PROP_THROTTLE, t)

            time.sleep(dt)

    def start(self):
        kb = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
        kb.start()

        try:
            self.loop()
        finally:
            self.running = False
            kb.stop()


if __name__ == "__main__":
    Bridge().start()