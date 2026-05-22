import time
from mo_grippers.grippers.base_gripper import BaseGripper


class FakeGripper(BaseGripper):
    def __init__(self, protocol=None):
        super().__init__(protocol)
        self.force = 50  # Default force percentage
        self.position = 500  # Default position (mid-range)
        self.speed = 50  # Default speed percentage
        self.state = "IDLE"

    def initialize(self):
        print("[FakeGripper] Initializing gripper...")
        time.sleep(0.5)
        self.state = "READY"
        print("[FakeGripper] Initialization complete.")

    def set_force(self, force):
        if 20 <= force <= 100:
            self.force = force
            print(f"[FakeGripper] Force set to {force}%.")
        else:
            print("[FakeGripper] Invalid force value. Must be between 20% and 100%.")

    def set_position(self, position):
        if 0 <= position <= 100:
            self.position = position
            print(f"[FakeGripper] Moving to position {position}...")
            time.sleep(0.5)
            print(f"[FakeGripper] Position reached: {self.position}.")
        else:
            print("[FakeGripper] Invalid position. Must be between 0 and 100.")

    def set_speed(self, speed):
        if 1 <= speed <= 100:
            self.speed = speed
            print(f"[FakeGripper] Speed set to {speed}%.")
        else:
            print("[FakeGripper] Invalid speed value. Must be between 1% and 100%.")

    def get_position(self):
        print(f"[FakeGripper] Current position: {self.position}")
        return self.position

    def get_state(self):
        print(f"[FakeGripper] Current state: {self.state}")
        return self.state

    def close(self):
        print("[FakeGripper] Closing gripper connection...")
        time.sleep(0.5)
        print("[FakeGripper] Connection closed.")


if __name__ == "__main__":
    # Create a fake gripper instance
    gripper = FakeGripper()

    # Test methods
    gripper.initialize()
    gripper.set_force(80)
    gripper.set_position(700)
    gripper.set_speed(60)
    gripper.get_position()
    gripper.get_state()
    gripper.close()
