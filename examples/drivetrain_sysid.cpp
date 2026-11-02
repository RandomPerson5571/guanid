#include "guanid/pros.hpp"
#include "pros/misc.hpp"

// Copy this file into your PROS project's src/ directory. No LemLib is needed.
// Call from a dedicated opcontrol test mode, using your existing motor groups:
//
//   drivetrainSysId(leftMotors, rightMotors, controller, {0.08255, 36.0 / 48.0});
//
// Geometry is wheel diameter in METRES and wheel revolutions per motor revolution.
// Configure motor reversal so positive voltage drives BOTH sides forward, set
// encoder units to degrees, and use matching gearing within each group.
// Suspend normal driver/autonomous motor commands while this function runs.
// Clear enough floor space for a ten-second test; reposition between tests.
//
// Hold L1, then press one direction to start ONE test:
//   Up    = quasistatic forward     Down = quasistatic reverse
//   Right = dynamic forward         Left = dynamic reverse
// Release L1 to stop; B exits. Run all four tests before analyzing the CSV.
// The output file is replaced each time this test mode is entered.
guanid::Result drivetrainSysId(pros::MotorGroup& left, pros::MotorGroup& right,
                               pros::Controller& controller,
                               const guanid::Geometry& geometry) {
    auto* csv = std::fopen("/usd/drivetrain_sysid.csv", "w");
    if (!csv) return guanid::Result::log_error;
    if (!guanid::writeRecordHeader(csv)) {
        std::fclose(csv);
        return guanid::Result::log_error;
    }

    guanid::ProsSysIdRoutine::Config config;
    config.run_id = pros::millis(); // The routine increments this for each test.
    config.ramp_volts_per_second = 1.0;
    config.step_volts = 7.0;
    config.max_volts = 12.0;
    config.duration_ms = 10000;
    config.interval_ms = 20;

    guanid::ProsSysIdRoutine routine(config, {
        guanid::ProsMotorChannel::linear("drive-left", left, geometry),
        guanid::ProsMotorChannel::linear("drive-right", right, geometry)
    }, [csv](const guanid::Record& row) { return guanid::writeRecord(csv, row); });

    // Require L1 to be released before arming the first or any subsequent test.
    bool armed = false;
    while (!pros::competition::is_disabled() &&
           !controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        const bool held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        // Poll every iteration so a direction pressed during a test is consumed.
        const bool up = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
        const bool down = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);
        const bool right_pressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
        const bool left_pressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
        if (!held) armed = true;
        if (armed && held && !routine.running() &&
            (up || down || right_pressed || left_pressed)) {
            armed = false;
            bool started;
            if (up || down)
                started = routine.startQuasistatic(up ? guanid::Direction::forward
                                                     : guanid::Direction::reverse);
            else
                started = routine.startDynamic(right_pressed ? guanid::Direction::forward
                                                             : guanid::Direction::reverse);
            if (!started) break;
        }
        const auto result = routine.update(held && !pros::competition::is_disabled() &&
            !controller.get_digital(pros::E_CONTROLLER_DIGITAL_B));
        if (result == guanid::Result::hardware_error || result == guanid::Result::log_error ||
            result == guanid::Result::invalid_config) break;
        pros::delay(5);
    }

    // Stop before closing the sink, including on disable or operator exit.
    const auto result = routine.cancel();
    const bool flushed = std::fflush(csv) == 0;
    const bool closed = std::fclose(csv) == 0;
    if ((!flushed || !closed) && result != guanid::Result::hardware_error)
        return guanid::Result::log_error;
    return result;
}

// After all four tests, copy /usd/drivetrain_sysid.csv from the SD card and run:
//   guanid_analyze drivetrain_sysid.csv --out drivetrain_gains.json
// The named drive-left / drive-right channels each produce kS, kV, and kA.
