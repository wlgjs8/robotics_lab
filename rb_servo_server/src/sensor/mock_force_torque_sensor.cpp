#include "rb_servo/sensor/mock_force_torque_sensor.hpp"

#include "rb_servo/core/clock.hpp"

namespace rb_servo {

MockForceTorqueSensor::MockForceTorqueSensor(std::string name) : name_(std::move(name)) {}

bool MockForceTorqueSensor::connect() { return true; }
bool MockForceTorqueSensor::start() { running_ = true; return true; }
void MockForceTorqueSensor::stop() { running_ = false; }

bool MockForceTorqueSensor::readLatest(Wrench6D& wrench_tcp, uint64_t& host_time_ns) {
    if (!running_) return false;
    wrench_tcp = wrench_;
    host_time_ns = nowSteadyNs();
    return true;
}

bool MockForceTorqueSensor::tare() {
    wrench_ = Wrench6D{};
    return true;
}

std::string MockForceTorqueSensor::name() const { return name_; }

void MockForceTorqueSensor::setWrench(const Wrench6D& wrench_tcp) { wrench_ = wrench_tcp; }

}  // namespace rb_servo
