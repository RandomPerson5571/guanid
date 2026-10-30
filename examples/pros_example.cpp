#include "guanid/pros.hpp"
#include "pros/misc.hpp"

// Call explicitly from an operator-controlled test mode, with your own groups.
// Configure every motor's direction, gear cartridge, encoder units (degrees),
// and coast brake mode first. Suspend drivetrain controllers during the test.
guanid::Result characterize(pros::MotorGroup& left, pros::MotorGroup& right,
                            pros::Controller& controller, guanid::Test test,
                            guanid::Direction direction) {
    if (!guanid::writeCsvHeader(stdout)) return guanid::Result::log_error;
    guanid::Config config;
    config.run_id = pros::millis(); // Use distinct IDs when appending repeated tests.
    const guanid::Geometry geometry{0.08255, 1.0}; // Example: 3.25-inch, direct drive.
    return guanid::run(left, right, geometry, test, direction, config,
        [&] { return controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1); },
        [](const guanid::Sample& sample) { return guanid::writeCsvSample(stdout, sample); },
        [](guanid::State state) {
            // Optional lifecycle logger. Keep state messages out of the CSV stream.
            (void)state;
        });
}
