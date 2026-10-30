#pragma once

#include "guanid/routine.hpp"
#include "pros/error.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

namespace guanid {

struct Geometry {
    double wheel_diameter_m;
    // Wheel revolutions per motor revolution, e.g. 36.0 / 48.0.
    double wheel_revolutions_per_motor_revolution = 1;

    bool valid() const {
        return std::isfinite(wheel_diameter_m) && wheel_diameter_m > 0 &&
               std::isfinite(wheel_revolutions_per_motor_revolution) &&
               wheel_revolutions_per_motor_revolution > 0;
    }
};

namespace detail {
inline bool readSide(pros::MotorGroup& motors, const Geometry& geometry, SideSample& sample) {
    const auto count = motors.size();
    if (count <= 0 || !geometry.valid()) return false;
    const double metres_per_revolution = 3.14159265358979323846 * geometry.wheel_diameter_m *
                                         geometry.wheel_revolutions_per_motor_revolution;
    sample = {};
    for (std::uint8_t index = 0; index < count; ++index) {
        if (motors.get_encoder_units(index) != pros::MotorUnits::degrees) return false;
        const double degrees = motors.get_position(index);
        const double rpm = motors.get_actual_velocity(index);
        const auto millivolts = motors.get_voltage(index);
        const SideSample motor{millivolts / 1000.0,
            degrees / 360.0 * metres_per_revolution, rpm / 60.0 * metres_per_revolution};
        if (!motor.valid()) return false;
        sample.applied_volts += motor.applied_volts / count;
        sample.position_m += motor.position_m / count;
        sample.velocity_mps += motor.velocity_mps / count;
    }
    return sample.valid();
}

struct ProsMechanism {
    pros::MotorGroup& left;
    pros::MotorGroup& right;
    const Geometry& geometry;

    std::uint32_t nowMs() const { return pros::millis(); }
    void sleepMs(std::uint32_t ms) const { pros::delay(ms); }
    bool setVoltage(std::int32_t millivolts) {
        // Do not short-circuit: always attempt both groups, including on stop.
        const auto l = left.move_voltage(millivolts);
        const auto r = right.move_voltage(millivolts);
        return l != PROS_ERR && r != PROS_ERR;
    }
    bool read(SideSample& l, SideSample& r) {
        return readSide(left, geometry, l) && readSide(right, geometry, r);
    }
};
} // namespace detail

// Blocking, single-test runner. keep_running must be a fast bool() callable
// checking an operator hold-to-run input and/or competition state. No background
// task is created. Callers must prevent other tasks from commanding these motors.
// sink is a fast bool(const Sample&) callable; false aborts the test.
template <typename KeepRunning, typename Sink, typename Observer = IgnoreState>
Result run(pros::MotorGroup& left, pros::MotorGroup& right,
           const Geometry& geometry, Test test, Direction direction,
           const Config& config, KeepRunning keep_running, Sink sink, Observer observer = {}) {
    detail::ProsMechanism mechanism{left, right, geometry};
    if (!geometry.valid()) {
        const bool stopped = mechanism.setVoltage(0);
        observer(State::idle);
        return stopped ? Result::invalid_config : Result::hardware_error;
    }
    return runRoutine(mechanism, test, direction, config, keep_running, sink, observer);
}

} // namespace guanid
