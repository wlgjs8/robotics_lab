#ifndef __mo_grippers_plaif_grippers_fake_gripper_h__
#define __mo_grippers_plaif_grippers_fake_gripper_h__

#include <plaif/mo-grippers/grippers/base-gripper.h>
#include <iostream>

namespace plaif
{
  class FakeGripper : public BaseGripper
  {
  private:
    int64_t position_ = 0;

  public:
    explicit FakeGripper() : BaseGripper() {}

    void initialize(bool should_wait) override
    {
      std::cout << "FakeGripper: Initialized\n";
      if (should_wait)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    void setPositionRaw(int64_t position) override
    {
      position_ = position;
    }

    void setPosition(uint16_t scaled) override
    {
      position_ = static_cast<int64_t>(scaled);
    }

    int64_t getPositionRaw() override
    {
      return position_;
    }

    uint16_t getPosition() override
    {
      return static_cast<uint16_t>(position_);
    }

    int16_t getState() override
    {
      return 0;
    }

    void setForce(int16_t force) override
    {
      std::cout << "FakeGripper: Setting force to " << force << "\n";
    }

    int16_t getForce() override
    {
      return 0;
    }

    void openFingers() override
    {
      position_ = 100;
      std::cout << "FakeGripper: Fingers opened\n";
    }

    void closeFingers() override
    {
      position_ = 0;
      std::cout << "FakeGripper: Fingers closed\n";
    }
  };
} // namespace plaif

#endif