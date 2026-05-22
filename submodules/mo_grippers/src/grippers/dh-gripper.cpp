#include <algorithm>
#include <plaif/mo-grippers/grippers/dh-gripper.h>

namespace plaif
{
  void DhGripper::initialize(bool should_wait)
  {
    protocol_->send(0x0100, 165);
    if (should_wait)
    {
      std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    std::cout << "DhGripper is initialized." << std::endl;
  }

  void DhGripper::setPositionRaw(int64_t position)
  {
    if (position < LOWER_LIMIT || position > UPPER_LIMIT)
    {
      throw std::out_of_range("Raw position must be between 0 and 1000!");
    }
    protocol_->send(0x0103, position);
  }

  void DhGripper::setPosition(uint16_t scaled)
  {
    if (scaled > 100)
    {
      throw std::out_of_range("Scaled position must be between 0 and 100!");
    }
    int64_t position = static_cast<int64_t>((UPPER_LIMIT - LOWER_LIMIT) / 100.0 * scaled);
    setPositionRaw(position);
  }

  int64_t DhGripper::getPositionRaw()
  {
    return static_cast<uint64_t>(protocol_->receive(0x0202));
  }

  uint16_t DhGripper::getPosition()
  {
    uint64_t raw_value = getPositionRaw();
    raw_value = std::min(static_cast<uint64_t>(UPPER_LIMIT),
                         std::max(static_cast<uint64_t>(LOWER_LIMIT), raw_value));
    return static_cast<uint16_t>(raw_value * 100 / (UPPER_LIMIT - LOWER_LIMIT));
  }

  void DhGripper::setForce(int16_t force)
  {
    if (force < 20 || force > 100)
    {
      throw std::out_of_range("Force must be between 20 and 100!");
    }
    protocol_->send(0x0101, force);
  }

  int16_t DhGripper::getForce()
  {
    return 0;
  }

  int16_t DhGripper::getState()
  {
    return static_cast<int16_t>(protocol_->receive(0x0201));
  }

  void DhGripper::openFingers()
  {
    setPositionRaw(UPPER_LIMIT);
  }

  void DhGripper::closeFingers()
  {
    setPositionRaw(LOWER_LIMIT);
  }
}
