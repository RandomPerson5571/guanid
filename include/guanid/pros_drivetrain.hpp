#pragma once

#include "guanid/pros.hpp"
#include "guanid/drivetrain.hpp"
#include "pros/misc.hpp"

namespace guanid {

// Also usable with DrivetrainTuner and custom controls. Configure degree encoders
// before capture. Geometry is SI; both groups must drive forward for positive V.
inline SysIdRoutine::Mechanism prosDrivetrainMechanism(
        pros::MotorGroup& left, pros::MotorGroup& right, Geometry geometry,
        ProsSysIdRoutine::CommandObserver command = {}) {
    return {
        [&left, &right, command](double volts) {
            const auto mv = static_cast<std::int32_t>(std::lround(volts * 1000));
            const auto l = left.move_voltage(mv), r = right.move_voltage(mv);
            if (command) command(volts);
            return l != PROS_ERR && r != PROS_ERR;
        },
        [&left, &right, geometry](SysIdLog& log) {
            SideSample l, r;
            if (!detail::readSide(left, geometry, l) || !detail::readSide(right, geometry, r)) return false;
            log.motor("drive-left").voltage(l.applied_volts).linearPosition(l.position_m).linearVelocity(l.velocity_mps);
            log.motor("drive-right").voltage(r.applied_volts).linearPosition(r.position_m).linearVelocity(r.velocity_mps);
            return true;
        }
    };
}

struct ProsTuningResult {
    Result capture = Result::cancelled;
    unsigned completed_profiles = 0;
    DrivetrainAnalysis analysis;
};

namespace detail {
struct EncoderRestore {
    pros::MotorGroup& left;
    pros::MotorGroup& right;
    std::vector<pros::MotorUnits> l, r;
    bool restored = false;
    EncoderRestore(pros::MotorGroup& left, pros::MotorGroup& right) : left(left), right(right) {
        for (std::uint8_t i = 0; i < left.size(); ++i) l.push_back(left.get_encoder_units(i));
        for (std::uint8_t i = 0; i < right.size(); ++i) r.push_back(right.get_encoder_units(i));
    }
    bool setup() {
        bool ok = !l.empty() && !r.empty() && &left != &right;
        for (std::uint8_t i = 0; i < l.size(); ++i)
            if (l[i] == pros::MotorUnits::invalid || left.set_encoder_units(pros::MotorUnits::degrees, i) == PROS_ERR) ok = false;
        for (std::uint8_t i = 0; i < r.size(); ++i)
            if (r[i] == pros::MotorUnits::invalid || right.set_encoder_units(pros::MotorUnits::degrees, i) == PROS_ERR) ok = false;
        return ok;
    }
    bool restore() {
        restored = true;
        bool ok = true;
        for (std::uint8_t i = 0; i < l.size(); ++i)
            if (l[i] != pros::MotorUnits::invalid && left.set_encoder_units(l[i], i) == PROS_ERR) ok = false;
        for (std::uint8_t i = 0; i < r.size(); ++i)
            if (r[i] != pros::MotorUnits::invalid && right.set_encoder_units(r[i], i) == PROS_ERR) ok = false;
        return ok;
    }
    ~EncoderRestore() { if (!restored) restore(); }
};
} // namespace detail

// Blocking opcontrol mode; no tasks or file ownership. L1 starts/holds the next
// profile; release cancels and retries it. Hold X without L1 to reposition with
// left-Y/right-X arcade. B exits. Requires release before every profile. Results
// are fitted in memory after all four profiles, even with no SD card/sink.
// permit must include your additional interlocks. Normal drive tasks must yield
// ownership until return. Callbacks are synchronous and must not throw on cleanup.
inline ProsTuningResult runDrivetrainTuning(
        pros::MotorGroup& left, pros::MotorGroup& right, pros::Controller& controller,
        Geometry geometry, DrivetrainConfig config = {}, DrivetrainTuner::Sink sink = {},
        std::function<bool()> permit = [] { return true; },
        ProsSysIdRoutine::CommandObserver command = {}) {
    detail::EncoderRestore encoders(left, right);
    auto mechanism = prosDrivetrainMechanism(left, right, geometry, std::move(command));
    // The stop guard outlives the session, and also covers manual repositioning.
    struct Stop { SysIdRoutine::Mechanism& m; ~Stop() { m.drive(0); } } stop{mechanism};
    if (!geometry.valid() || &left == &right) config.quasistatic.interval_ms = 0;
    DrivetrainTuner tuner(config, mechanism, prosRuntime(), std::move(sink));
    ProsTuningResult out;
    if (!config.valid()) { out.capture = mechanism.drive(0) ? Result::invalid_config : Result::hardware_error; return out; }
    if (!encoders.setup()) { mechanism.drive(0); out.capture = Result::hardware_error; return out; }
    const auto enabled = [&] {
        return controller.is_connected() && !pros::competition::is_disabled() &&
            !pros::competition::is_autonomous() && permit && permit();
    };
    constexpr const char* prompts[] = {"1/4 L1: Ramp Fwd", "2/4 L1: Ramp Rev",
        "3/4 L1: Step Fwd", "4/4 L1: Step Rev", "Done: see results"};
    unsigned shown = 0;
    controller.print(0, 0, "%-18s", prompts[0]);
    bool armed = false, repositioning = false;
    while (enabled() && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && !tuner.complete()) {
        const bool held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        if (!held && !tuner.running()) armed = true;
        const bool manual = !held && !tuner.running() && controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        if (manual) {
            const double forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
            const double turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;
            const double maxMv = 1000 * std::min({config.reposition_max_volts,
                config.quasistatic.max_volts, config.dynamic.max_volts});
            const auto l = left.move_voltage(static_cast<std::int32_t>(std::lround(std::clamp(forward+turn, -1.0, 1.0)*maxMv)));
            const auto r = right.move_voltage(static_cast<std::int32_t>(std::lround(std::clamp(forward-turn, -1.0, 1.0)*maxMv)));
            repositioning = true;
            if (l == PROS_ERR || r == PROS_ERR) { out.capture = Result::hardware_error; break; }
        } else if (repositioning) {
            repositioning = false;
            if (!mechanism.drive(0)) { out.capture = Result::hardware_error; break; }
        }
        if (armed && held && !tuner.running()) {
            armed = false;
            if (!tuner.startNext()) { out.capture = tuner.result(); break; }
        }
        out.capture = tuner.update(held && enabled() && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_B));
        if (!held && !tuner.running()) armed = true;
        if (tuner.completedProfiles() != shown) {
            shown = tuner.completedProfiles();
            controller.print(0, 0, "%-18s", prompts[shown]);
        }
        if (out.capture == Result::hardware_error || out.capture == Result::invalid_config || out.capture == Result::log_error) break;
        pros::delay(5);
    }
    const auto cancelled = tuner.cancel();
    const bool stopped = mechanism.drive(0);
    const bool restored = encoders.restore();
    if (cancelled == Result::hardware_error || !stopped || !restored)
        out.capture = Result::hardware_error;
    else if (out.capture == Result::completed && !tuner.complete()) out.capture = Result::cancelled;
    out.completed_profiles = tuner.completedProfiles();
    if (out.capture == Result::completed) out.analysis = tuner.analyzeResults();
    return out;
}
} // namespace guanid
