#pragma once

#include "guanid/routine.hpp"
#include "guanid/sysid.hpp"
#include "guanid/public_units.hpp"
#include "pros/error.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

namespace guanid {

inline SysIdRoutine::Runtime prosRuntime() {
    return {[] { return pros::millis(); }, [](std::uint32_t ms) { pros::delay(ms); }};
}

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
inline bool readSide(pros::AbstractMotor& motors, const Geometry& geometry, SideSample& sample) {
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

// A named channel references an existing PROS Motor or MotorGroup. It owns no
// hardware. Configure reversal and degree encoder units on the motors first.
class ProsMotorChannel {
public:
    // Preferred public factories. Geometry is inches; arm offsets are degrees.
    template <typename Motor>
    static ProsMotorChannel linearInches(std::string name, Motor& motors, double wheelDiameterIn,
                                        double wheelTurnsPerMotorTurn = 1, int driveDirection = 1) {
        return linear(std::move(name), motors, {wheelDiameterIn * metersPerInch, wheelTurnsPerMotorTurn}, driveDirection);
    }
    template <typename Motor>
    static ProsMotorChannel angularDegrees(std::string name, Motor& motors,
                                          double outputTurnsPerMotorTurn = 1,
                                          double horizontalOffsetDeg = 0, int driveDirection = 1) {
        return angular(std::move(name), motors, outputTurnsPerMotorTurn,
                       horizontalOffsetDeg * radiansPerDegree, driveDirection);
    }
    static ProsMotorChannel linear(std::string name, pros::Motor& motor,
                                   Geometry geometry, int drive_direction = 1) {
        return {std::move(name), motor, geometry, Units::linear, 0, drive_direction};
    }
    static ProsMotorChannel linear(std::string name, pros::MotorGroup& motors,
                                   Geometry geometry, int drive_direction = 1) {
        return {std::move(name), motors, geometry, Units::linear, 0, drive_direction};
    }
    static ProsMotorChannel angular(std::string name, pros::Motor& motor,
                                    double output_revolutions_per_motor_revolution = 1,
                                    double horizontal_offset_radians = 0, int drive_direction = 1) {
        return {std::move(name), motor, {2, output_revolutions_per_motor_revolution},
                Units::angular, horizontal_offset_radians, drive_direction};
    }
    static ProsMotorChannel angular(std::string name, pros::MotorGroup& motors,
                                    double output_revolutions_per_motor_revolution = 1,
                                    double horizontal_offset_radians = 0, int drive_direction = 1) {
        return {std::move(name), motors, {2, output_revolutions_per_motor_revolution},
                Units::angular, horizontal_offset_radians, drive_direction};
    }

private:
    friend class ProsSysIdRoutine;
    std::string name_;
    pros::AbstractMotor* motors_;
    Geometry geometry_;
    Units units_;
    double offset_;
    int drive_direction_;

    ProsMotorChannel(std::string name, pros::AbstractMotor& motors, Geometry geometry,
                     Units units, double offset, int drive_direction)
        : name_(std::move(name)), motors_(&motors), geometry_(geometry), units_(units),
          offset_(offset), drive_direction_(drive_direction) {}

    bool valid() const {
        MotorSample name_check;
        name_check.name = name_;
        name_check.voltage(0).linearPosition(0).linearVelocity(0);
        return name_check.valid() && geometry_.valid() && std::isfinite(offset_) &&
               (drive_direction_ == 1 || drive_direction_ == -1) && motors_->size() > 0;
    }
    bool drive(double volts) const {
        const auto mv = static_cast<std::int32_t>(std::lround(volts * 1000));
        return motors_->move_voltage(mv * drive_direction_) != PROS_ERR;
    }
    bool log(SysIdLog& log) const {
        SideSample sample;
        if (!detail::readSide(*motors_, geometry_, sample)) return false;
        auto& row = log.motor(name_);
        row.voltage(sample.applied_volts);
        if (units_ == Units::linear)
            row.linearPosition(sample.position_m).linearVelocity(sample.velocity_mps);
        else
            row.angularPosition(sample.position_m + offset_).angularVelocity(sample.velocity_mps);
        return true;
    }
};

// Only PROS motor channels can command hardware through this interface. Sink,
// observer and operator permit callbacks still run synchronously on the caller.
class ProsSysIdRoutine {
public:
    using Config = SysIdRoutine::Config;
    using Direction = guanid::Direction;
    using Sink = SysIdRoutine::Sink;
    using Observer = SysIdRoutine::Observer;

    ProsSysIdRoutine(Config config, std::vector<ProsMotorChannel> channels, Sink sink,
                     Observer observer = {})
        : routine_(validatedConfig(config, channels), mechanism(channels), prosRuntime(),
                   std::move(sink), std::move(observer)) {}

    bool running() const { return routine_.running(); }
    Result result() const { return routine_.result(); }
    State state() const { return routine_.state(); }
    bool startQuasistatic(Direction direction) { return routine_.startQuasistatic(direction); }
    bool startDynamic(Direction direction) { return routine_.startDynamic(direction); }
    Result update(bool permit = true) { return routine_.update(permit); }
    Result cancel() { return routine_.cancel(); }
    template <typename Permit>
    Result quasistatic(Direction direction, Permit permit) { return routine_.quasistatic(direction, permit); }
    template <typename Permit>
    Result dynamic(Direction direction, Permit permit) { return routine_.dynamic(direction, permit); }

private:
    SysIdRoutine routine_;

    static Config validatedConfig(Config config, const std::vector<ProsMotorChannel>& channels) {
        bool valid = !channels.empty();
        for (std::size_t i = 0; i < channels.size(); ++i) {
            if (!channels[i].valid()) valid = false;
            for (std::size_t j = 0; j < i; ++j)
                if (channels[i].name_ == channels[j].name_ || channels[i].motors_ == channels[j].motors_)
                    valid = false;
        }
        // Let the engine perform its normal invalid-config stop on every channel.
        if (!valid) config.interval_ms = 0;
        return config;
    }
    static SysIdRoutine::Mechanism mechanism(const std::vector<ProsMotorChannel>& channels) {
        return {
            [channels](double volts) {
                bool ok = true;
                for (const auto& channel : channels)
                    if (!channel.drive(volts)) ok = false; // Always attempt every motor, including stop.
                return ok;
            },
            [channels](SysIdLog& log) {
                bool ok = true;
                for (const auto& channel : channels)
                    if (!channel.log(log)) ok = false;
                return ok;
            }
        };
    }
};

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
