#ifndef __mo_grippers_plaif_grippers_base_gripper_h__
#define __mo_grippers_plaif_grippers_base_gripper_h__

#include <thread>
#include <atomic>
#include <chrono>
#include <plaif/mo-grippers/protocols/base-protocol.h>

namespace plaif
{
  class BaseGripper
  {
  protected:
    BaseProtocol *protocol_;
    std::atomic<bool> is_running_;
    std::thread read_thread_;
    std::atomic<uint64_t> current_position_;

    void readLoop(uint16_t rate)
    {
      while (is_running_)
      {
        current_position_ = getPosition();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / rate));
      }
    }

  public:
    explicit BaseGripper(BaseProtocol *protocol) : protocol_(protocol), is_running_(false), current_position_(0) {}
    explicit BaseGripper() : protocol_(nullptr), is_running_(false), current_position_(0) {}

    virtual ~BaseGripper()
    {
      close();
    }

    virtual void initialize(bool should_wait) = 0;
    virtual void setPositionRaw(int64_t position) = 0;
    virtual void setPosition(uint16_t scaled) = 0;
    virtual int64_t getPositionRaw() = 0;
    virtual uint16_t getPosition() = 0;
    virtual void setForce(int16_t force) = 0;
    virtual int16_t getForce() = 0;
    virtual int16_t getState() = 0;
    virtual void openFingers() = 0;
    virtual void closeFingers() = 0;
    uint16_t getCurrentPosition()
    {
      return current_position_;
    }

    void close()
    {
      is_running_ = false;
      if (read_thread_.joinable())
      {
        read_thread_.join();
      }
    }

    void startThread(uint16_t read_rate = 100)
    {
      is_running_ = true;
      read_thread_ = std::thread(&BaseGripper::readLoop, this, read_rate);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000 / read_rate));
    }
  };
}

#endif
