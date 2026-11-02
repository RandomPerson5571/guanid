#include "guanid/pros.hpp"
#include "guanid/analysis.hpp"
#include "pros/misc.hpp"

// Call explicitly from your own opcontrol test mode. No subsystems, scheduler,
// global tasks or framework are required. Configure motor reversal, cartridges,
// encoder units (degrees), and gearing before entering. Other motor controllers
// must be suspended. Release L1 to stop. B exits this mode.
// Never return to normal driver control while a characterization button is held.
void sysIdTestMode(pros::MotorGroup& left, pros::MotorGroup& right,
                   pros::Controller& controller, bool turn_in_place = false) {
    constexpr double wheelDiameterIn = 3.25; // direct drive
    auto* csv = std::fopen("/usd/guanid.csv", "w");
    if (!csv) return;
    if (!guanid::writeRecordHeader(csv)) { std::fclose(csv); return; }
    guanid::ProsSysIdRoutine::Config config;
    // WPILib-style defaults: 1 V/s, 7 V, 10 seconds. Reduce for your mechanism.
    config.run_id = pros::millis();
    guanid::ProsSysIdRoutine routine(config, {
        guanid::ProsMotorChannel::linearInches("drive-left", left, wheelDiameterIn, 1, turn_in_place ? -1 : 1),
        guanid::ProsMotorChannel::linearInches("drive-right", right, wheelDiameterIn)
    }, [csv](const guanid::Record& row) { return guanid::writePublicRecord(csv, row); });

    while (!pros::competition::is_disabled() &&
           !controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        const bool held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        if (held && !routine.running()) {
            // Four independent tests, each explicitly started by the operator.
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
                routine.startQuasistatic(guanid::Direction::forward);
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
                routine.startQuasistatic(guanid::Direction::reverse);
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
                routine.startDynamic(guanid::Direction::forward);
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
                routine.startDynamic(guanid::Direction::reverse);
        }
        // Add arm/elevator travel limits here when using those mechanisms.
        routine.update(held && !pros::competition::is_disabled());
        pros::delay(5);
    }
    routine.cancel();
    // A successful fprintf may still fail during buffered flush. Check both.
    const bool flushed = std::fflush(csv) == 0;
    const bool closed = std::fclose(csv) == 0;
    if (!flushed || !closed) std::fprintf(stderr,"guanid: capture write failed\n");
}

// Pure C++ in-memory alternative: use this object as your sink instead of CSV.
// Keep it alive across four separately invoked tests; clear before a new session.
// Limit memory explicitly on the V5. Analyze only after stopping the motors.
struct InCodeCapture {
    std::vector<guanid::Record> samples;
    bool operator()(const guanid::Record& row) {
        if (samples.size() >= 5000) return false; // log_error stops the test
        samples.push_back(row);
        return true;
    }
    guanid::PublicAnalysis gains(const std::string& motor, guanid::Model model) const {
        guanid::AnalysisConfig settings;
        settings.model = model;
        return guanid::analyzePublic(samples,motor,settings);
    }
};

// For a flywheel/arm, replace the drivetrain channels with:
//   guanid::ProsMotorChannel::angular("arm", armMotors, 1.0/7.0, horizontalOffset)
// For an elevator, use a linear channel with the spool diameter as geometry.
// Arm encoder zero must be homed relative to horizontal before entering test mode.
// Then analyze with Model::arm / Model::elevator. For example:
//   const auto fit = capture.gains("arm", guanid::Model::arm);
//   if (fit.ok) {
//       const auto& g = fit.gains;
//       guanid::ArmFeedforward ff(g.kS, g.kV, g.kA, g.kG);
//       double volts = ff.calculate(targetAngleRadians, targetRadps, targetRadps2);
//       armMotors.move_voltage(std::lround(std::clamp(volts, -12.0, 12.0)*1000));
//   }
