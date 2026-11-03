#pragma once

#include "guanid/guanid.hpp"
#include "guanid/track_width.hpp"
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace guanid {

enum class Units { linear, angular };
enum class Model { simple, elevator, arm };

struct MotorSample {
    std::string name;
    Units units = Units::linear;
    double volts = 0, position = 0, velocity = 0;
    unsigned fields = 0;
    MotorSample& voltage(double v) { volts = v; fields |= 1; return *this; }
    MotorSample& linearPosition(double m) { position = m; units = Units::linear; fields |= 2; return *this; }
    MotorSample& linearVelocity(double mps) { velocity = mps; fields |= 4; return *this; }
    MotorSample& angularPosition(double radians) { position = radians; units = Units::angular; fields |= 8; return *this; }
    MotorSample& angularVelocity(double radps) { velocity = radps; fields |= 16; return *this; }
    bool valid() const {
        return !name.empty() && name.find_first_not_of(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.") == std::string::npos &&
            ((fields == 7 && units == Units::linear) || (fields == 25 && units == Units::angular)) &&
            std::isfinite(volts) && std::abs(volts) <= 12 &&
            std::isfinite(position) && std::isfinite(velocity);
    }
};

class SysIdLog {
public:
    std::vector<MotorSample> motors;
    // Complete each chain before adding another motor (vector references may move).
    MotorSample& motor(const std::string& name) {
        for (auto& sample : motors) if (sample.name == name) return sample;
        motors.push_back({});
        motors.back().name = name;
        return motors.back();
    }
    bool valid() const {
        if (motors.empty()) return false;
        for (const auto& sample : motors) if (!sample.valid()) return false;
        return true;
    }
};

struct Record {
    std::uint32_t run_id = 0;
    Test test = Test::quasistatic;
    Direction direction = Direction::forward;
    double time_s = 0;
    MotorSample motor;
};

inline bool writeRecordHeader(std::FILE* file) {
    return file && std::fprintf(file,
        "run_id,test,direction,time_s,motor,units,voltage,position,velocity\n") >= 0;
}
inline bool writeRecord(std::FILE* file, const Record& row) {
    if (!file || !row.motor.valid() || !std::isfinite(row.time_s) || row.time_s < 0 ||
        (row.test != Test::quasistatic && row.test != Test::dynamic) ||
        (row.direction != Direction::forward && row.direction != Direction::reverse)) return false;
    return std::fprintf(file, "%lu,%s,%s,%.9f,%s,%s,%.9f,%.12f,%.12f\n",
        static_cast<unsigned long>(row.run_id), row.test == Test::dynamic ? "dynamic" : "quasistatic",
        row.direction == Direction::forward ? "forward" : "reverse", row.time_s,
        row.motor.name.c_str(), row.motor.units == Units::linear ? "m" : "rad",
        row.motor.volts, row.motor.position, row.motor.velocity) >= 0;
}

// All callbacks run on the calling task. Drive receives VOLTS, not millivolts.
// Callbacks must be fast; drive(0) must not throw. This object owns no motors/tasks.
// Portable, synchronous capture engine. Mechanism callbacks provide hardware I/O;
// Runtime supplies the clock, and Sink decides where each channel record goes.
// A routine runs one profile/direction at a time; the caller selects all four
// tests separately and keeps every captured object's dependencies alive.
class SysIdRoutine {
public:
    using Direction = guanid::Direction;
    struct Config : guanid::Config {
        Config() { duration_ms = 10000; ramp_volts_per_second = 1; step_volts = 7; max_volts = 12; }
    };
    // drive receives volts, including zero for cleanup. log fills one snapshot
    // with stable channel names and SI measurements. Both report failure as false;
    // neither callback may block indefinitely because cancellation is cooperative.
    struct Mechanism {
        std::function<bool(double)> drive;
        std::function<bool(SysIdLog&)> log;
    };
    struct Runtime {
        std::function<std::uint32_t()> nowMs;
        std::function<void(std::uint32_t)> sleepMs;
    };
    using Sink = std::function<bool(const Record&)>;
    using Observer = std::function<void(State)>;

    SysIdRoutine(Config config, Mechanism mechanism, Runtime runtime, Sink sink,
                 Observer observer = {})
        : config_(config), mechanism_(std::move(mechanism)), runtime_(std::move(runtime)),
          sink_(std::move(sink)), observer_(std::move(observer)) {}
    SysIdRoutine(const SysIdRoutine&) = delete;
    SysIdRoutine& operator=(const SysIdRoutine&) = delete;
    ~SysIdRoutine() { if (running_) finish(Result::cancelled); }

    bool running() const { return running_; }
    Result result() const { return result_; }
    State state() const { return running_ ? testState(test_, direction_) : State::idle; }
    bool startQuasistatic(Direction direction) { return start(Test::quasistatic, direction); }
    bool startDynamic(Direction direction) { return start(Test::dynamic, direction); }
    Result cancel() { return running_ ? finish(Result::cancelled) : result_; }

    // Call every interval_ms or faster. A late update skips missed samples.
    // permit must include operator enable, competition state and any travel limits.
    Result update(bool permit = true) {
        if (!running_) return result_;
        struct UnwindStop {
            SysIdRoutine& routine;
            bool returned = false;
            ~UnwindStop() { if (!returned) routine.cancel(); }
        } guard{*this};
        // Each ordinary return disarms the unwinding guard.
        auto done = [&](Result result) { guard.returned = true; return result; };
        if (!permit) return done(finish(Result::cancelled));
        // Unsigned subtraction supports a millisecond clock wrapping during a run.
        // Config restricts durations so elapsed-time comparisons remain meaningful.
        const auto elapsed = runtime_.nowMs() - start_;
        if (elapsed >= config_.duration_ms) return done(finish(Result::completed));
        if (elapsed < next_) return done(result_);
        // Read before applying the next command so each record describes the
        // voltage already active during measurement, including initial zero output.
        SysIdLog log;
        if (!mechanism_.log(log) || !log.valid()) return done(finish(Result::hardware_error));
        if (runtime_.nowMs() - start_ >= config_.duration_ms) return done(finish(Result::completed));
        // Keep channel names and units stable throughout a capture.
        if (channels_.empty()) channels_ = log.motors;
        if (channels_.size() != log.motors.size()) return done(finish(Result::log_error));
        for (std::size_t i = 0; i < channels_.size(); ++i) {
            if (channels_[i].name != log.motors[i].name || channels_[i].units != log.motors[i].units)
                return done(finish(Result::log_error));
            if (!sink_(Record{run_id_, test_, direction_, elapsed / 1000.0, log.motors[i]}))
                return done(finish(Result::log_error));
        }
        const auto now = runtime_.nowMs() - start_;
        if (now >= config_.duration_ms) return done(finish(Result::completed));
        if (!mechanism_.drive(commandVolts(config_, test_, direction_, now)))
            return done(finish(Result::hardware_error));
        const auto after = runtime_.nowMs() - start_;
        if (after >= config_.duration_ms) return done(finish(Result::completed));
        // Skip missed deadlines rather than emitting a burst of stale samples.
        // Derivatives later use recorded elapsed time, not an assumed fixed period.
        next_ = std::min(config_.duration_ms, (after / config_.interval_ms + 1) * config_.interval_ms);
        return done(result_);
    }

    template <typename Permit>
    Result quasistatic(Direction direction, Permit permit) { return run(Test::quasistatic, direction, permit); }
    template <typename Permit>
    Result dynamic(Direction direction, Permit permit) { return run(Test::dynamic, direction, permit); }

private:
    Config config_;
    Mechanism mechanism_;
    Runtime runtime_;
    Sink sink_;
    Observer observer_;
    bool running_ = false;
    Result result_ = Result::completed;
    Test test_ = Test::quasistatic;
    Direction direction_ = Direction::forward;
    std::uint32_t start_ = 0, next_ = 0, run_id_ = 0, sequence_ = 0;
    std::vector<MotorSample> channels_;

    // All ordinary stop paths converge here. Even invalid setup attempts zero
    // output; inability to stop takes precedence over the original exit reason.
    Result finish(Result result) {
        running_ = false;
        const bool stopped = mechanism_.drive && mechanism_.drive(0);
        result_ = stopped ? result : Result::hardware_error;
        if (observer_) observer_(State::idle);
        return result_;
    }
    bool start(Test test, Direction direction) {
        // A start request never silently replaces an active test. Cancel it and
        // require another explicit start, preventing accidental profile changes.
        if (running_) { finish(Result::cancelled); return false; }
        if (!config_.valid() || !mechanism_.drive || !mechanism_.log || !runtime_.nowMs ||
            !runtime_.sleepMs || !sink_ ||
            (direction != Direction::forward && direction != Direction::reverse)) {
            finish(Result::invalid_config); return false;
        }
        if (!mechanism_.drive(0)) { finish(Result::hardware_error); return false; }
        test_ = test;
        direction_ = direction;
        start_ = runtime_.nowMs();
        next_ = 0;
        run_id_ = config_.run_id + sequence_++;
        channels_.clear();
        result_ = Result::running;
        running_ = true;
        struct StartGuard {
            SysIdRoutine& routine;
            bool returned = false;
            ~StartGuard() { if (!returned) routine.cancel(); }
        } guard{*this};
        if (observer_) observer_(state());
        guard.returned = true;
        return true;
    }
    template <typename Permit>
    Result run(Test test, Direction direction, Permit permit) {
        if (!start(test, direction)) return result_;
        // Stack unwinding also stops outputs on host builds with exceptions enabled.
        struct Guard { SysIdRoutine& routine; ~Guard() { routine.cancel(); } } guard{*this};
        while (running_) {
            update(permit());
            if (running_) {
                const auto elapsed = runtime_.nowMs() - start_;
                if (elapsed < next_) runtime_.sleepMs(next_ - elapsed);
            }
        }
        return result_;
    }
};

struct SimpleMotorFeedforward {
    double kS = 0, kV = 0, kA = 0;
    double calculate(double velocity, double acceleration = 0) const {
        return kS * ((velocity > 0) - (velocity < 0)) + kV * velocity + kA * acceleration;
    }
};
struct ElevatorFeedforward : SimpleMotorFeedforward {
    double kG = 0;
    ElevatorFeedforward(double s, double v, double a, double g) : SimpleMotorFeedforward{s,v,a}, kG(g) {}
    double calculate(double velocity, double acceleration = 0) const {
        return SimpleMotorFeedforward::calculate(velocity, acceleration) + kG;
    }
};
struct ArmFeedforward : SimpleMotorFeedforward {
    double kG = 0;
    ArmFeedforward(double s, double v, double a, double g) : SimpleMotorFeedforward{s,v,a}, kG(g) {}
    // Zero radians is horizontal, positive angles/voltages point upward.
    double calculate(double angle_radians, double velocity, double acceleration = 0) const {
        return SimpleMotorFeedforward::calculate(velocity, acceleration) + kG * std::cos(angle_radians);
    }
};

} // namespace guanid
