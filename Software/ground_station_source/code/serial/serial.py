"""
Manage serial connection
"""

from PyQt6.QtSerialPort import QSerialPortInfo, QSerialPort
from PyQt6.QtCore import QObject, QIODevice, pyqtSignal
from .payload_sim import PayloadSim

class SerialManager(QObject):

    error_catch = pyqtSignal(str)
    fatal_catch = pyqtSignal(str)
    print_catch = pyqtSignal(str)
    recv_data_signal = pyqtSignal(str)

    def __init__(self, parent=None):

        super().__init__(parent)

        self._physical_driver = QSerialPort(self)
        self._sim_driver = PayloadSim(self)
        self._active_serial = self._physical_driver
        self._physical_driver.setBaudRate(57600)
        self._physical_driver.readyRead.connect(self.recv_data)
        self._physical_driver.errorOccurred.connect(self.handle_serial_error)

        self._sim_driver.readyRead.connect(self.recv_data)
        self._sim_driver.errorOccurred.connect(self.handle_serial_error)
        self._ports = []
        self._port_name = None
        self._port_desc = None

        self._serial_buffer = b""
    
    # Search for open ports
    def search_ports(self) -> list:
        phys_ports = QSerialPortInfo.availablePorts()
        self._ports.clear()
        for p in phys_ports:
            self._ports.append({
                "driver": self._physical_driver,
                "info": p,
                "name": p.portName(),
                "desc": p.description()
            })
        self._ports.append({
            "driver": self._sim_driver,
            "info": None,
            "name": "SIM",
            "desc": "Internal Payload Simulator"
        })
        return [(item["name"], item["desc"]) for item in self._ports]

    # Set port from list
    def set_port(self, idx):
        if idx < 0 or idx >= len(self._ports):
            self.error_catch.emit("CODE ERROR: Selected port index beyond available list")
            return
        interface = self._ports[idx]
        self._active_serial = interface["driver"]
        self._port_name = interface["name"]
        self._port_desc = interface["desc"]
        if interface["info"] is not None:
             self._active_serial.setPort(interface["info"])
        self.print_catch.emit(f"Port {self._port_name} {self._port_desc} selected")

    # Open connection on port
    def open_port(self) -> bool:
        if self._active_serial.isOpen():
            self.error_catch.emit("ERROR: Port is already open")
            return False

        # Both QSerialPort and PayloadSim accept OpenModeFlag
        if self._port_name is None:
            self.error_catch.emit("ERROR: No port selected")
            return False
        elif self._active_serial.open(QIODevice.OpenModeFlag.ReadWrite):
            self.print_catch.emit(f"Port opened on {self._port_name}")
            return True
        else:
            self.error_catch.emit("ERROR: Port could not be opened")
            return False

    # Close connection on port
    def close_port(self) -> bool:
        if self._port_name is None:
            self.error_catch.emit("ERROR: No port selected")
            return False
        elif self._active_serial.isOpen():
            self._active_serial.close()
            if self._active_serial.isOpen():
                self.error_catch.emit("ERROR: Port could not be closed")
                return False
            else:
                self.print_catch.emit("Port closed")
                return True
        else:
            self.error_catch.emit("ERROR: Port is already closed")
            return False
    
    # Check if port is open
    def is_port_open(self) -> bool:
        return self._active_serial.isOpen()

    # Handle serial connection error
    def handle_serial_error(self, error):
        if error == QSerialPort.SerialPortError.NoError:
            return
        self.fatal_catch.emit(f"FATAL SERIAL ERROR: {error}")
        if self._active_serial.isOpen():
            self._active_serial.close()

    # Send data through serial port
    def send_data(self, msg):
        print(f"Sent message: {msg}")  # Debug print for sending message
        if self._active_serial.isOpen():
            try:
                msg = msg + "\r"
                if self._active_serial is self._sim_driver:
                    self.print_catch.emit(f"DEBUG: {msg}")
                self._active_serial.write(msg.encode())
                return 1
            except Exception as e:
                self.error_catch.emit(f"ERROR: CANNOT SEND DATA - {e}")
                return 0
        else:
            self.error_catch.emit("ERROR: Port is closed, cannot send data")
            return 0

    # Process received data
    def recv_data(self):
        self._serial_buffer += self._active_serial.readAll().data()
        while b'\r' in self._serial_buffer:
            line, self._serial_buffer = self._serial_buffer.split(b'\r', 1)
            if not line:
                continue # Skip empty lines
            msg = line.decode().strip()
            if msg:
                print(f"Received message: {msg}")  # Debug print for processed message
                self.recv_data_signal.emit(msg)