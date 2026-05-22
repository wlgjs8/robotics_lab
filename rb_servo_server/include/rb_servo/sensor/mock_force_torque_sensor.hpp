#pragma once

#include "rb_servo/sensor/i_force_torque_sensor.hpp"

namespace rb_servo {

class MockForceTorqueSensor final : public IForceTorqueSensor {
public:
    explicit MockForceTorqueSensor(std::string name = "mock_ft");

    bool connect() override;
    bool start() override;
    void stop() override;

    bool readLatest(Wrench6D& wrench_tcp, uint64_t& host_time_ns) override;
    bool tare() override;
    std::string name() const override;

    void setWrench(const Wrench6D& wrench_tcp);

private:
    std::string name_;
    Wrench6D wrench_{};
    bool running_ = false;
};

}  // namespace rb_servo
