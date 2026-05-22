import time
from mo_grippers.protocols.modbus import ModbusProtocol
from mo_grippers.grippers.base_gripper import BaseGripper


LOWER_LIMIT = 0
UPPER_LIMIT = 1000


class DhGripper(BaseGripper):
    protocol: ModbusProtocol

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, slave_id=1):
        super().__init__(
            ModbusProtocol(port=port, baudrate=baudrate, slave_id=slave_id)
        )

    def initialize(self, should_wait=False):
        response = self.protocol.send(0x0100, 165)
        if should_wait:
            time.sleep(5.0)
        print("DhGripper is initialized.")
        return response

    def set_force(self, force):
        if 20 <= force <= 100:
            return self.protocol.send(0x0101, force)
        else:
            raise ValueError("Force must be between 20% and 100%")

    def set_position_raw(self, position):
        assert (
            LOWER_LIMIT <= position <= UPPER_LIMIT
        ), "Position must be between 0 and 1000"
        return self.protocol.send(0x0103, int(position))

    def set_position(self, scaled):
        assert 0 <= scaled <= 100, "Scaled position should be between 0 and 100"
        position = (UPPER_LIMIT - LOWER_LIMIT) / 100 * scaled
        return self.set_position_raw(position)

    def get_position_raw(self):
        return self.protocol.receive(0x0202)

    def get_position(self):
        position = self.get_position_raw()
        return position * 100 / (UPPER_LIMIT - LOWER_LIMIT)

    def get_state(self):
        return self.protocol.receive(0x0201)

    def open_fingers(self):
        return self.set_position_raw(UPPER_LIMIT)

    def close_fingers(self):
        return self.set_position_raw(LOWER_LIMIT)
