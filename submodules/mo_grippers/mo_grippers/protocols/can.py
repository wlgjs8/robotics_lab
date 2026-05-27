import can
from mo_grippers.protocols.base_protocol import BaseProtocol


class CanProtocol(BaseProtocol):
    def __init__(self, channel="can0", bitrate=500000):
        self.bus = can.interface.Bus(channel=channel, bustype="socketcan", bitrate=bitrate)

    def send(self, data, arbitration_id):
        message = can.Message(data=data, arbitration_id=arbitration_id, is_extended_id=False)
        self.bus.send(message)

    def receive(self, timeout=1):
        return self.bus.recv(timeout)

    def close(self):
        self.bus.shutdown()
