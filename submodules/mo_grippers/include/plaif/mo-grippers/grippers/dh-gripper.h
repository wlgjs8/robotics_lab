#ifndef __mo_grippers_plaif_grippers_dh_gripper_h__
#define __mo_grippers_plaif_grippers_dh_gripper_h__

#include <plaif/mo-grippers/grippers/base-gripper.h>
#include <plaif/mo-grippers/protocols/modbus.h>
#include <iostream>
#include <stdexcept>

namespace plaif
{
  class DhGripper : public BaseGripper
  {
  private:
    static constexpr uint16_t LOWER_LIMIT = 0;
    static constexpr uint16_t UPPER_LIMIT = 1000;

  public:
    explicit DhGripper(const std::string &port = "/dev/ttyUSB0", int baudrate = 115200, int slave_id = 1)
        : BaseGripper(new ModbusProtocol(port)) {}

    void initialize(bool should_wait = false) override;
    void setPositionRaw(int64_t position) override;
    void setPosition(uint16_t scaled) override;
    int64_t getPositionRaw() override;
    uint16_t getPosition() override;
    void setForce(int16_t force) override;
    int16_t getForce() override;
    void openFingers() override;
    void closeFingers() override;
    int16_t getState() override;

    static inline uint16_t MOVING = 0;
    static inline uint16_t REACHED = 1;
    static inline uint16_t GRIPPED = 2;
    static inline uint16_t DROPPED = 3;
  };
}

#endif
