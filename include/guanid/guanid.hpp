#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace guanid {

enum class Test { quasistatic, dynamic };
enum class Direction { forward, reverse };
enum class Result { completed, cancelled, invalid_config, hardware_error, log_error, running };
enum class State { idle, quasistatic_forward, quasistatic_reverse, dynamic_forward, dynamic_reverse };

inline State testState(Test test, Direction direction) {
    if (test == Test::dynamic)
        return direction == Direction::forward ? State::dynamic_forward : State::dynamic_reverse;
    return direction == Direction::forward ? State::quasistatic_forward : State::quasistatic_reverse;
}

inline const char* stateName(State state) {
    switch (state) {
        case State::quasistatic_forward: return "quasistatic-forward";
        case State::quasistatic_reverse: return "quasistatic-reverse";
        case State::dynamic_forward: return "dynamic-forward";
        case State::dynamic_reverse: return "dynamic-reverse";
        default: return "none";
    }
}

struct Config {
    std::uint32_t duration_ms = 3000;
    std::uint32_t interval_ms = 20;
    double ramp_volts_per_second = 0.5;
    double step_volts = 3.0;
    double max_volts = 6.0;
    std::uint32_t run_id = 0; // Choose a unique ID for repeated captures in one file.

    bool valid() const {
        return duration_ms > 0 && duration_ms < 0x80000000u &&
               interval_ms > 0 && interval_ms <= duration_ms &&
               std::isfinite(ramp_volts_per_second) && ramp_volts_per_second > 0 &&
               std::isfinite(step_volts) && step_volts > 0 &&
               std::isfinite(max_volts) && max_volts > 0 && max_volts <= 12;
    }
};

// Call only with a validated Config and nonnegative elapsed time.
inline double commandVolts(const Config& config, Test test, Direction direction,
                           std::uint32_t elapsed_ms) {
    if (elapsed_ms >= config.duration_ms) return 0;
    const double magnitude = test == Test::dynamic
        ? config.step_volts : config.ramp_volts_per_second * (elapsed_ms / 1000.0);
    return std::min(magnitude, config.max_volts) *
           (direction == Direction::forward ? 1.0 : -1.0);
}

struct SideSample {
    double applied_volts = 0;
    double position_m = 0;
    double velocity_mps = 0;

    bool valid() const {
        return std::isfinite(applied_volts) && std::abs(applied_volts) <= 12 &&
               std::isfinite(position_m) && std::isfinite(velocity_mps);
    }
};

struct Sample {
    Test test = Test::quasistatic;
    Direction direction = Direction::forward;
    std::uint32_t elapsed_ms = 0;
    double commanded_volts = 0;
    SideSample left;
    SideSample right;
    std::uint32_t run_id = 0;
};

// One row contains both sides. A sample's command is the command active when
// telemetry was read, not the command that will be issued for the next interval.
inline bool writeCsvHeader(std::FILE* stream) {
    return stream && std::fprintf(stream,
        "test,direction,elapsed_s,commanded_volts,left_applied_volts,left_position_m,"
        "left_velocity_mps,right_applied_volts,right_position_m,right_velocity_mps,run_id\n") >= 0;
}

inline bool writeCsvSample(std::FILE* stream, const Sample& sample) {
    return stream && std::fprintf(stream, "%s,%s,%.6f,%.6f,%.6f,%.9f,%.9f,%.6f,%.9f,%.9f,%lu\n",
        sample.test == Test::dynamic ? "dynamic" : "quasistatic",
        sample.direction == Direction::forward ? "forward" : "reverse",
        sample.elapsed_ms / 1000.0, sample.commanded_volts,
        sample.left.applied_volts, sample.left.position_m, sample.left.velocity_mps,
        sample.right.applied_volts, sample.right.position_m, sample.right.velocity_mps,
        static_cast<unsigned long>(sample.run_id)) >= 0;
}

// Gains use volts, metres/second, and metres/second squared.
// At zero velocity kS contributes 0; choose a nonzero setpoint to command breakaway.
struct Feedforward {
    double kS = 0;
    double kV = 0;
    double kA = 0;

    double calculate(double velocity_mps, double acceleration_mps2 = 0) const {
        const double sign = (velocity_mps > 0) - (velocity_mps < 0);
        return kS * sign + kV * velocity_mps + kA * acceleration_mps2;
    }
};

} // namespace guanid
