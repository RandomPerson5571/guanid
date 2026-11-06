#pragma once

#include "guanid/public_units.hpp"

namespace guanid {

struct DrivetrainConfig {
    SysIdRoutine::Config quasistatic;
    SysIdRoutine::Config dynamic;
    AnalysisConfig analysis; // SI thresholds; drivetrain fits always use the simple model.
    OutputUnits output;
    double track_width_m = 0; // Supplied geometry, never calibrated here. Zero disables angular estimates.
    double limit_volts = 12;
    // Manual repositioning is also bounded by both capture profiles' max_volts.
    // Zero disables manual movement; limit_volts above is only a model estimate input.
    double reposition_max_volts = 3;
    std::size_t max_records = 4000;
    DrivetrainConfig() {
        quasistatic.duration_ms = 3000;
        quasistatic.ramp_volts_per_second = 2;
        dynamic.duration_ms = 2000;
    }
    bool valid() const {
        return quasistatic.valid() && dynamic.valid() && analysis.valid() &&
            analysis.model == Model::simple && output.valid() &&
            std::isfinite(track_width_m) && track_width_m >= 0 &&
            std::isfinite(limit_volts) && limit_volts > 0 && limit_volts <= 12 &&
            std::isfinite(reposition_max_volts) && reposition_max_volts >= 0 &&
            reposition_max_volts <= 12 && max_records > 0;
    }
};

struct DriveLimits {
    bool valid = false;
    double velocity = 0, acceleration = 0, braking_acceleration = 0;
    bool angular_valid = false;
    double angular_velocity = 0;
};

// SI fits in, selectable units out. Full-reverse braking is the initial magnitude
// at the common straight-line maximum speed, not a constant stopping constraint.
inline DriveLimits estimateDriveLimits(const Fit& left, const Fit& right,
                                      double trackWidthM = 0, OutputUnits output = {}, double volts = 12) {
    if (!left.ok || !right.ok || !output.valid() || !std::isfinite(volts) || volts <= 0 || volts > 12 ||
        !std::isfinite(trackWidthM) || trackWidthM < 0) return {};
    const Gains gains[] = {left.gains, right.gains};
    double speed[2], accel[2], brake[2];
    for (int i = 0; i < 2; ++i) {
        const auto g = gains[i];
        if (!std::isfinite(g.kS) || g.kS < 0 || g.kS >= volts ||
            !std::isfinite(g.kV) || g.kV <= 0 || !std::isfinite(g.kA) || g.kA <= 0) return {};
        speed[i] = (volts - g.kS) / g.kV;
        accel[i] = (volts - g.kS) / g.kA;
    }
    const double v = std::min(speed[0], speed[1]);
    for (int i = 0; i < 2; ++i)
        brake[i] = (volts + gains[i].kS + gains[i].kV * v) / gains[i].kA;
    const auto scale = output.scale(Units::linear);
    DriveLimits out{true, v / scale, std::min(accel[0], accel[1]) / scale,
        std::min(brake[0], brake[1]) / scale, trackWidthM > 0,
        trackWidthM > 0 ? (speed[0] + speed[1]) / trackWidthM / output.scale(Units::angular) : 0};
    if (!std::isfinite(out.velocity) || !std::isfinite(out.acceleration) ||
        !std::isfinite(out.braking_acceleration) || !std::isfinite(out.angular_velocity)) return {};
    return out;
}

struct DrivetrainAnalysis {
    bool ok = false;
    PublicAnalysis left, right;
    Gains lateral; // Mean of left/right simple-model gains, in selected linear units.
    Gains angular; // Ideal turn estimate, in selected angular units; see angular_valid.
    bool angular_valid = false;
    DriveLimits limits;
};

// Portable four-profile capture session. Supply SI drive-left/drive-right channels.
// Owns records and routines, borrows callback dependencies, creates no tasks.
// update/start/cancel expose the same tests to custom controls as the built-in loop.
class DrivetrainTuner {
public:
    using Sink = SysIdRoutine::Sink;
    DrivetrainTuner(DrivetrainConfig config, SysIdRoutine::Mechanism mechanism,
                    SysIdRoutine::Runtime runtime, Sink sink = {}, SysIdRoutine::Observer observer = {})
        : config_(config), sink_(std::move(sink)),
          ramp_(profileConfig(config, true), mechanism, runtime,
                [this](const Record& row) { return capture(row); }, observer),
          step_(profileConfig(config, false), std::move(mechanism), std::move(runtime),
                [this](const Record& row) { return capture(row); }, std::move(observer)) {
        if (config_.valid()) {
            const auto samples = [](const SysIdRoutine::Config& p) -> std::uint64_t {
                return p.duration_ms / p.interval_ms + (p.duration_ms % p.interval_ms != 0);
            };
            // Two sides and two directions per profile type. Bound allocation
            // before narrowing to size_t on the robot's 32-bit target.
            const auto count = 4 * (samples(config_.quasistatic) + samples(config_.dynamic));
            records_.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(count, config_.max_records)));
        }
    }
    DrivetrainTuner(const DrivetrainTuner&) = delete;
    DrivetrainTuner& operator=(const DrivetrainTuner&) = delete;
    ~DrivetrainTuner() { cancel(); }

    bool running() const { return active_ && active_->running(); }
    Result result() const { return result_; }
    State state() const { return running() ? active_->state() : State::idle; }
    unsigned completedProfiles() const {
        unsigned count = 0;
        for (unsigned bit = 1; bit <= 8; bit <<= 1) if (completed_ & bit) ++count;
        return count;
    }
    bool complete() const { return completed_ == 15; }
    const std::vector<Record>& records() const { return records_; } // SI, completed profiles only when idle.
    bool startQuasistatic(Direction d) { return start(Test::quasistatic, d); }
    bool startDynamic(Direction d) { return start(Test::dynamic, d); }
    bool startNext() {
        for (unsigned i = 0; i < 4; ++i)
            if (!(completed_ & (1u << i)))
                return start(i < 2 ? Test::quasistatic : Test::dynamic,
                             i % 2 == 0 ? Direction::forward : Direction::reverse);
        return false;
    }
    Result update(bool permit = true) {
        if (running()) {
            result_ = active_->update(permit);
            if (!active_->running()) settle();
        }
        return result_;
    }
    Result cancel() {
        if (running()) {
            result_ = active_->cancel();
            settle();
        }
        return result_;
    }
    DrivetrainAnalysis analyzeResults() const {
        DrivetrainAnalysis out;
        if (!complete() || running() || !config_.valid() || result_ != Result::completed) return out;
        out.left = analyzePublic(records_, "drive-left", config_.analysis, config_.output);
        out.right = analyzePublic(records_, "drive-right", config_.analysis, config_.output);
        out.ok = out.left.si.ok && out.right.si.ok;
        if (!out.ok) return out;
        const auto l = out.left.gains(), r = out.right.gains();
        out.lateral = {(l.kS+r.kS)/2, (l.kV+r.kV)/2, (l.kA+r.kA)/2, 0};
        out.angular_valid = config_.track_width_m > 0;
        if (out.angular_valid)
            out.angular = lateralToAngular(out.lateral,
                config_.track_width_m / config_.output.scale(Units::linear), config_.output);
        out.limits = estimateDriveLimits(out.left.si, out.right.si, config_.track_width_m,
                                         config_.output, config_.limit_volts);
        return out;
    }

private:
    DrivetrainConfig config_;
    Sink sink_;
    std::vector<Record> records_;
    SysIdRoutine ramp_, step_;
    SysIdRoutine* active_ = nullptr;
    Result result_ = Result::completed;
    unsigned completed_ = 0, pending_ = 0;
    std::size_t checkpoint_ = 0;
    std::uint32_t sequence_ = 0, run_id_ = 0;
    static SysIdRoutine::Config profileConfig(const DrivetrainConfig& c, bool ramp) {
        auto profile = ramp ? c.quasistatic : c.dynamic;
        if (!c.valid()) profile.interval_ms = 0;
        return profile;
    }
    bool start(Test test, Direction direction) {
        if (running()) { cancel(); return false; }
        if ((test != Test::quasistatic && test != Test::dynamic) ||
            (direction != Direction::forward && direction != Direction::reverse)) return false;
        const unsigned index = (test == Test::dynamic ? 2 : 0) + (direction == Direction::reverse ? 1 : 0);
        if (completed_ & (1u << index)) return false;
        pending_ = 1u << index;
        checkpoint_ = records_.size();
        run_id_ = config_.quasistatic.run_id + sequence_++;
        active_ = test == Test::quasistatic ? &ramp_ : &step_;
        const bool started = test == Test::quasistatic ? active_->startQuasistatic(direction)
                                                       : active_->startDynamic(direction);
        result_ = active_->result();
        return started;
    }
    bool capture(Record row) {
        if (row.motor.units != Units::linear ||
            (row.motor.name != "drive-left" && row.motor.name != "drive-right") ||
            records_.size() >= config_.max_records) return false;
        row.run_id = run_id_;
        records_.push_back(row);
        return !sink_ || sink_(row);
    }
    void settle() {
        if (result_ == Result::completed) completed_ |= pending_;
        else records_.resize(checkpoint_); // A retry must never fit an incomplete attempt.
    }
};
} // namespace guanid
