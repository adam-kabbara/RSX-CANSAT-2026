"""
Manage serial connection
"""

from PyQt6.QtSerialPort import QSerialPortInfo, QSerialPort
from PyQt6.QtCore import QObject, QIODevice, pyqtSignal
from enum import Enum, auto

class SerialPortToggleStatus(Enum):
    COULD_NOT_CLOSE  = auto()
    CLOSED           = auto()
    OPENED           = auto()
    COULD_NOT_OPEN   = auto()
    NO_PORT_SELECTED = auto()

class SerialManager(QObject):

    error_catch = pyqtSignal(str)
    recv_data_str = pyqtSignal(str)

    def __init__(self, parent=None):

        super().__init__(parent)

        self._serial = QSerialPort(self)
        self._serial.setBaudRate(57600)
        self._serial.readyRead.connect(self.recv_data)
        self._serial.errorOccurred.connect(self.handle_serial_error)
    
    # Search for open ports
    @staticmethod
    def search_ports():
        return [(p.portName(), p.description(), p) for p in QSerialPortInfo.availablePorts()]
    
    # Open/close ports
    def open_close_port(self, port_info: QSerialPortInfo):
        if self._serial.isOpen() is True:
            self._serial.close()
            if self._serial.isOpen():
                return SerialPortToggleStatus.COULD_NOT_CLOSE
            else:
                return SerialPortToggleStatus.CLOSED
        elif port_info is not None:
            self._serial.setPort(self.__PORT_SELECTED_INFO)
            if self._serial.open(QIODevice.OpenModeFlag.ReadWrite):
                return SerialPortToggleStatus.OPENED
            else:
                return SerialPortToggleStatus.COULD_NOT_OPEN
        else:
            return SerialPortToggleStatus.NO_PORT_SELECTED
    
    # Handle serial connection error
    def handle_serial_error(self, error):
        if error == QSerialPort.SerialPortError.ResourceError:
            self.error_catch.emit("FATAL SERIAL ERROR: Device disconnected")
            self._serial.close()
        
        elif error == QSerialPort.SerialPortError.OpenError:
            self.error_catch.emit("FATAL SERIAL ERROR: Could not open port")

        elif error == QSerialPort.SerialPortError.DeviceNotFoundError:
            self.error_catch.emit("FATAL SERIAL ERROR: Device not found")
            self._serial.close()

        elif error != QSerialPort.SerialPortError.NoError:
            self.error_catch.emit(f"FATAL SERIAL ERROR: {error} detected")

    # Send data through serial port
    def send_data(self, msg):
        if self._serial.isOpen() is True:
            try:
                msg = msg + "\n"
                self._serial.write(msg.encode())
                return 1
            except Exception as e:
                self.error_catch.emit(f"ERROR: CANNOT SEND DATA - {e}")
                self._serial.close()
        else:
            self.error_catch.emit("ERROR: Port is not open, cannot send data")

    # Process received data
    def recv_data(self, write_to_logfile=False):
        while self.__serial.canReadLine():
            msg = self.__serial.readLine().data().decode().strip()
            if msg:
                self.recv_data_str.emit(msg)
