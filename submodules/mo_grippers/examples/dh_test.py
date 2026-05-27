import time
from mo_grippers.grippers import DhGripper

gripper = DhGripper(port="/dev/ttyUSB1")
gripper.initialize(should_wait=True)
gripper.start_thread(read_rate=100) # 100Hz

gripper.close_fingers()
for _ in range(10):
    print("Current position: ", gripper.current_position)
    time.sleep(0.1)

gripper.open_fingers()
for _ in range(10):
    print("Current position: ", gripper.current_position)
    time.sleep(0.1)

print("Closing read thread and client connection!!")
gripper.close()
