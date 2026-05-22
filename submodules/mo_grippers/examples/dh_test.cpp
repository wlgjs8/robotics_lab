#include <chrono>
#include <iostream>
#include <thread>
#include <plaif/mo-grippers/grippers/dh-gripper.h>

using namespace plaif;

int main()
{
  DhGripper gripper("/dev/ttyUSB1");
  gripper.initialize(true);
  gripper.startThread(100);

  gripper.closeFingers();
  for (int i = 0; i < 10; i++)
  {
    std::cout << "Current position: " << gripper.getCurrentPosition() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  gripper.openFingers();
  for (int i = 0; i < 10; i++)
  {
    std::cout << "Current position: " << gripper.getCurrentPosition() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Closing read thread and client connection!!" << std::endl;
  gripper.close();

  return 0;
}
