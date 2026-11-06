#include "guanid/pros_drivetrain.hpp"

// Call from a dedicated opcontrol mode; other motor-writing tasks must yield.
// L1: start/hold next test, release: cancel/retry; X + sticks: reposition; B: exit.
// Geometry here is inches and wheel turns per motor turn. No LemLib required.
guanid::Result drivetrainSysId(pros::MotorGroup& left, pros::MotorGroup& right,
                               pros::Controller& controller,
                               double wheelDiameterIn, double wheelTurnsPerMotorTurn = 1) {
    guanid::DrivetrainConfig config;
    config.quasistatic.run_id = pros::millis();
    config.quasistatic.ramp_volts_per_second = 2;
    config.quasistatic.duration_ms = 3000;
    config.dynamic.step_volts = 7;
    config.dynamic.duration_ms = 2000;
    config.output = {guanid::LinearUnit::inches, guanid::AngularUnit::degrees};
    config.track_width_m = 13.5 * guanid::metersPerInch; // Your measured/configured width.
    auto* csv = std::fopen("/usd/drivetrain_sysid.csv", "w");
    if (csv && !guanid::writeRecordHeader(csv)) { std::fclose(csv); csv = nullptr; }
    const auto result = guanid::runDrivetrainTuning(left, right, controller,
        {wheelDiameterIn * guanid::metersPerInch, wheelTurnsPerMotorTurn}, config,
        [&](const guanid::Record& row) {
            // SD failures do not discard the in-memory fit.
            if (csv && !guanid::writePublicRecord(csv, row, config.output)) {
                std::fclose(csv); csv = nullptr;
            }
            return true;
        });
    if (csv) std::fclose(csv);
    if (result.analysis.ok) {
        const auto& fit = result.analysis;
        std::printf("Lateral kS %.6f V; kV %.6f V s/in; kA %.6f V s^2/in\n",
            fit.lateral.kS, fit.lateral.kV, fit.lateral.kA);
        if (fit.angular_valid)
            std::printf("Estimated angular kS %.6f V; kV %.6f V s/deg; kA %.6f V s^2/deg\n",
                fit.angular.kS, fit.angular.kV, fit.angular.kA);
        if (fit.limits.valid)
            std::printf("Limits: %.3f in/s; accel %.3f in/s^2; initial braking %.3f in/s^2\n",
                fit.limits.velocity, fit.limits.acceleration, fit.limits.braking_acceleration);
    }
    return result.capture;
}
