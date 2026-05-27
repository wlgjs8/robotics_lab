from pymodbus.client.sync import ModbusSerialClient
from mo_grippers.protocols.base_protocol import BaseProtocol


class ModbusProtocol(BaseProtocol):
    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=115200,
        parity="N",
        stopbits=1,
        bytesize=8,
        slave_id=1,
        timeout=1,
    ):
        self.client = ModbusSerialClient(
            method="rtu",
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=timeout,
        )
        self.slave_id = slave_id

        if not self.client.connect():
            raise ConnectionError("Unable to connect via Modbus")

    def send(self, address, value):
        response = self.client.write_register(address, value, unit=self.slave_id)
        return response

    def receive(self, address):
        response = self.client.read_holding_registers(address, 1, unit=self.slave_id)
        return response.registers[0] if response and not response.isError() else None

    def close(self):
        self.client.close()
