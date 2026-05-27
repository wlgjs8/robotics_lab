#pragma once

#include <cstdint>
#include <string>

#include "rb_servo/core/types.hpp"

namespace rb_servo {

class IForceTorqueSensor {
public:
    virtual ~IForceTorqueSensor() = default;

    virtual bool connect() = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;

    virtual bool readLatest(Wrench6D& wrench_tcp, uint64_t& host_time_ns) = 0;
    virtual bool tare() = 0;
    virtual std::string name() const = 0;
};

}  // namespace rb_servo
