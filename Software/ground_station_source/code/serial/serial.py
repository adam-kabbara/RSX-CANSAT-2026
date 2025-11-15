"""
Manage serial connection
"""

from PyQt6.QtSerialPort import QSerialPortInfo, QSerialPort
from PyQt6.QtCore import QObject, QIODevice, pyqtSignal

class SerialManager(QObject):

    error_catch = pyqtSignal(str)
    fatal_catch = pyqtSignal(str)
    print_catch = pyqtSignal(str)
    recv_data_signal = pyqtSignal(str)

    def __init__(self, parent=None):

        super().__init__(parent)

        self._serial = QSerialPort(self)
        self._serial.setBaudRate(57600)
        self._serial.readyRead.connect(self.recv_data)
        self._serial.errorOccurred.connect(self.handle_serial_error)
        self._ports  = None
        self._port_name = None
        self._port_desc = None
    
    # Search for open ports
    def search_ports(self) -> bool:
        self._ports = QSerialPortInfo.availablePorts()
        return [(p.portName(), p.description()) for p in self._ports]
    
    # Set port from list
    def set_port(self, idx):
        if len(self._ports) > 0:
            if idx > len(self._ports):
                self.error_catch.emit("CODE ERROR: Selected port index beyond available list")
                return
            self._serial.setPort(self._ports[idx])
            if self._serial.portName() == "":
                self.error_catch.emit(f"ERROR: Could not set port to {self._ports[idx].portName()}")
            else:
                self._port_name = self._ports[idx].portName()
                self._port_desc = self._ports[idx].description()
                self.print_catch.emit(f"Port {self._port_name} {self._port_desc} selected")
        else:
            self.error_catch.emit("ERROR: Port list is empty")
    
    # Open connection on port
    def open_port(self) -> bool:
        if self._serial.portName() == "":
            self.error_catch.emit("ERROR: No port selected")
            return False
        elif self._serial.isOpen():
            self.error_catch.emit("ERROR: Port is already open")
            return False
        else:
            if self._serial.open(QIODevice.OpenModeFlag.ReadWrite):
                self.print_catch.emit(f"Port opened on {self._port_name} {self._port_desc}")
                return True
            else:
                self.error_catch.emit("ERROR: Port could not be opened")
                return False

    # Close connection on port
    def close_port(self) -> bool:
        if self._serial.portName() == "":
            self.error_catch.emit("ERROR: No port selected")
            return False
        if self._serial.isOpen():
            self._serial.close()
            if self._serial.isOpen():
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
        if self._serial.isOpen():
            return True
        return False
    
    # Handle serial connection error
    def handle_serial_error(self, error):
        self.fatal_catch.emit(f"FATAL SERIAL ERROR: {error}")
        self._serial.close()

    # Send data through serial port
    def send_data(self, msg):
        if self._serial.isOpen() is True:
            try:
                msg = msg + "\n"
                self._serial.write(msg.encode())
                return 1
            except Exception as e:
                self.error_catch.emit(f"ERROR: CANNOT SEND DATA - {e}")
                return 0
        else:
            self.error_catch.emit("ERROR: Port is closed, cannot send data")
            return 0
        
    # Process received data
    def recv_data(self):
        while self.__serial.canReadLine():
            msg = self.__serial.readLine().data().decode().strip()
            if msg:
                self.recv_data_signal.emit(msg)
