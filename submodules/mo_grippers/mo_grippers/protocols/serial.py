import serial
from mo_grippers.protocols.base_protocol import BaseProtocol

class SerialProtocol(BaseProtocol):
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

    def send(self, data):
        if isinstance(data, str):
            data = data.encode()
        self.serial.write(data)

    def receive(self, size=100):
        return self.serial.read(size)

    def close(self):
        self.serial.close()
