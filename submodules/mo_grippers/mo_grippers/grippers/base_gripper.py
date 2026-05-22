import threading
import time
from mo_grippers.protocols.base_protocol import BaseProtocol


class BaseGripper:
    protocol: BaseProtocol

    def __init__(self, protocol):
        self.protocol = protocol
        self.current_position = None
        self._is_running = False

    def initialize(self):
        raise NotImplementedError("Must be implemented by subclasses")

    def set_force(self, force):
        raise NotImplementedError("Must be implemented by subclasses")

    def set_position_raw(self, position):
        raise NotImplementedError("Must be implemented by subclasses")

    def set_position(self, scaled):
        raise NotImplementedError("Must be implemented by subclasses")

    def get_force(self):
        raise NotImplementedError("Must be implemented by subclasses")

    def get_position_raw(self):
        raise NotImplementedError("Must be implemented by subclasses")

    def get_position(self, scaled_position):
        raise NotImplementedError("Must be implemented by subclasses")

    def open_fingers(self):
        raise NotImplementedError("Must be implemented by subclasses")

    def close_fingers(self):
        raise NotImplementedError("Must be implemented by subclasses")

    def close(self):
        self._is_running = False
        if hasattr(self, "_read_thread"):
            self._read_thread.join()
        self.protocol.close()

    def start_thread(self, read_rate=100):
        self._is_running = True
        self._read_thread = threading.Thread(target=self._read_loop, args=(read_rate,))
        self._read_thread.start()

    def _read_loop(self, rate):
        while self._is_running:
            self.current_position = self.get_position()
            time.sleep(1 / rate)
