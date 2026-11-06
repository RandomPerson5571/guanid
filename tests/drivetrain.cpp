#include "guanid/pros_drivetrain.hpp"
#include <cstdlib>

using namespace guanid;
#define check(ok) do { if (!(ok)) { std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #ok); std::abort(); } } while (false)
static bool near(double a, double b) { return std::abs(a-b) < 1e-8; }

int main() {
    // Every combination must represent the same physical voltage model.
    const Gains si{1, 2, 3, 0};
    for (auto linear : {LinearUnit::meters, LinearUnit::inches}) {
        for (auto angular : {AngularUnit::radians, AngularUnit::degrees}) {
            const OutputUnits units{linear, angular};
            const auto l = gainsInUnits(si, Units::linear, units);
            const auto a = lateralToAngular(l, .4/units.scale(Units::linear), units);
            check(near(a.kS, 1));
            check(near(a.kV, .4*units.scale(Units::angular)));
            check(near(a.kA, .6*units.scale(Units::angular)));
            check(near(a.calculate(Model::simple, 0, 2/units.scale(Units::angular), 3/units.scale(Units::angular)), 3.6));
            PublicAnalysis arm;
            arm.si.model = Model::arm; arm.si.units = Units::angular;
            arm.si.gains = {0, 0, 0, 2}; arm.output = units;
            check(near(arm.calculate(3.14159265358979323846/units.scale(Units::angular), 0, 0), -2));
            Fit fit; fit.ok = true; fit.gains = si;
            const auto limits = estimateDriveLimits(fit, fit, .4, units);
            check(limits.valid && limits.angular_valid);
            check(near(limits.velocity*units.scale(Units::linear), 5.5));
            check(near(limits.angular_velocity*units.scale(Units::angular), 27.5));
            check(near(limits.braking_acceleration*units.scale(Units::linear), 8));
        }
    }
    check(std::isnan(lateralToAngular(si, 0).kV));
    Fit bad; check(!estimateDriveLimits(bad, bad).valid);

    // CSV selection is independent on each axis; the underlying sample stays SI.
    for (auto linear : {LinearUnit::meters, LinearUnit::inches}) {
        for (auto angular : {AngularUnit::radians, AngularUnit::degrees}) {
            const OutputUnits units{linear, angular};
            auto* file = std::tmpfile(); check(file);
            Record row; row.motor.name = "wheel";
            row.motor.voltage(2).linearPosition(1).linearVelocity(2);
            check(writePublicRecord(file, row, units));
            row.motor = {}; row.motor.name = "arm";
            row.motor.voltage(2).angularPosition(1).angularVelocity(2);
            check(writePublicRecord(file, row, units));
            std::rewind(file);
            char buffer[1024]{};
            check(std::fread(buffer, 1, sizeof(buffer)-1, file) > 0);
            const std::string csv(buffer);
            check(csv.find(std::string(",wheel,")+units.name(Units::linear)+",") != std::string::npos);
            check(csv.find(std::string(",arm,")+units.name(Units::angular)+",") != std::string::npos);
            std::fclose(file);
        }
    }
    // DriveLab observer reports requests separately from measured applied volts.
    {
        pros::Motor motor;
        std::vector<double> commands;
        ProsSysIdRoutine routine({}, {ProsMotorChannel::linearInches("motor", motor, 3)},
            [](const Record&) { return true; }, {}, [&](double v) { commands.push_back(v); });
        check(routine.startDynamic(Direction::forward));
        routine.update(); routine.cancel();
        check(commands.size() == 3 && commands[0] == 0 && commands[1] == 7 && commands[2] == 0);
    }
    DrivetrainConfig config;
    config.quasistatic.duration_ms = 60; config.dynamic.duration_ms = 40;
    std::uint32_t clock = 0;
    double volts = 0;
    const SysIdRoutine::Runtime runtime{[&] { return clock; }, [&](std::uint32_t ms) { clock += ms; }};
    const SysIdRoutine::Mechanism mechanism{
        [&](double v) { volts = v; return true; },
        [&](SysIdLog& log) {
            for (const char* side : {"drive-left", "drive-right"})
                log.motor(side).voltage(volts).linearPosition(clock*.001).linearVelocity(1);
            return true;
        }
    };
    {
        DrivetrainTuner tuner(config, mechanism, runtime);
        check(tuner.startNext());
        tuner.update(); clock += 20; tuner.update();
        check(tuner.records().size() == 4);
        check(tuner.update(false) == Result::cancelled && volts == 0);
        check(tuner.records().empty() && tuner.completedProfiles() == 0);
        for (int profile = 0; profile < 4; ++profile) {
            check(tuner.startNext());
            while (tuner.running()) { tuner.update(); clock += 20; }
            check(volts == 0 && tuner.result() == Result::completed);
        }
        check(tuner.complete() && tuner.completedProfiles() == 4);
        check(tuner.records().size() == 20);
        check(!tuner.startNext());
        check(!tuner.analyzeResults().ok); // Constant velocity cannot identify acceleration.
        check(tuner.records()[0].run_id != tuner.records()[6].run_id);
    }
    {
        auto small = config; small.max_records = 1;
        DrivetrainTuner tuner(small, mechanism, runtime);
        check(tuner.startNext());
        check(tuner.update() == Result::log_error && tuner.records().empty() && volts == 0);
    }
    {
        auto invalid = config; invalid.dynamic.interval_ms = 0;
        DrivetrainTuner tuner(invalid, mechanism, runtime);
        check(!tuner.startNext() && tuner.result() == Result::invalid_config && volts == 0);
    }
    // Identify a simulated drivetrain end to end, then convert the fitted result.
    {
        DrivetrainConfig settings;
        settings.output = {LinearUnit::meters, AngularUnit::radians};
        settings.track_width_m = .4;
        settings.analysis.median_window = 1;
        settings.analysis.derivative_window = 5;
        settings.analysis.min_velocity = .1;
        settings.analysis.trim_start_s = .1;
        std::uint32_t now = 0;
        double voltage = 0, velocity = 0, position = 0;
        const auto advance = [&](std::uint32_t ms) {
            for (std::uint32_t i = 0; i < ms; ++i) {
                const double sign = (velocity > 0) - (velocity < 0);
                const double friction = std::abs(velocity) < 1e-8
                    ? std::clamp(voltage, -.4, .4) : .4*sign;
                velocity += (voltage-friction-2*velocity)/.3*.001;
                position += velocity*.001;
                ++now;
            }
        };
        DrivetrainTuner tuner(settings, {
            [&](double v) { voltage = v; return true; },
            [&](SysIdLog& log) {
                for (const char* side : {"drive-left", "drive-right"})
                    log.motor(side).voltage(voltage).linearPosition(position).linearVelocity(velocity);
                return true;
            }
        }, {[&] { return now; }, advance});
        for (int i = 0; i < 4; ++i) {
            check(tuner.startNext());
            while (tuner.running()) { tuner.update(); advance(20); }
            advance(2000); // Settle between profiles.
        }
        const auto fit = tuner.analyzeResults();
        check(fit.ok && fit.angular_valid && fit.limits.valid);
        check(std::abs(fit.lateral.kS-.4) < .05);
        check(std::abs(fit.lateral.kV-2) < .05);
        check(std::abs(fit.lateral.kA-.3) < .03);
        check(near(fit.angular.kV, fit.lateral.kV*.2));
    }
    // Real default controller loop with simulated buttons/clock: cancel a profile,
    // release, retry, complete all four, and restore every motor's original units.
    {
        pros::test_clock = 0;
        pros::MotorGroup left, right;
        left.readings.resize(2); left.readings[1].units = pros::MotorUnits::rotations;
        pros::Controller controller;
        controller.digital = [](pros::controller_digital_e_t b) {
            const auto t = pros::test_clock;
            return b == pros::E_CONTROLLER_DIGITAL_L1 &&
                ((t >= 5 && t < 25) || (t >= 30 && t < 100) ||
                 (t >= 110 && t < 180) || (t >= 190 && t < 240) || (t >= 250 && t < 300));
        };
        auto result = runDrivetrainTuning(left, right, controller, {.1, 1}, config, {},
                                         [] { return pros::test_clock < 400; });
        check(result.capture == Result::completed && result.completed_profiles == 4);
        check(left.commands.back() == 0 && right.commands.back() == 0);
        check(left.readings[0].units == pros::MotorUnits::degrees && left.readings[1].units == pros::MotorUnits::rotations);
    }
    // A failed stop still attempts the other group and restores encoder units.
    {
        pros::MotorGroup left, right;
        left.fail_stop = true;
        left.readings[0].units = pros::MotorUnits::rotations;
        pros::Controller controller;
        const auto result = runDrivetrainTuning(left, right, controller, {.1, 1}, config, {}, [] { return false; });
        check(result.capture == Result::hardware_error);
        check(!right.commands.empty() && right.commands.back() == 0);
        check(left.readings[0].units == pros::MotorUnits::rotations);
    }
    // Reposition, then disconnect: manual outputs must be stopped as well.
    {
        pros::test_clock = 0;
        pros::MotorGroup left, right;
        pros::Controller controller;
        controller.digital = [](pros::controller_digital_e_t b) { return b == pros::E_CONTROLLER_DIGITAL_X; };
        controller.analog = [](pros::controller_analog_e_t) { return 50; };
        const auto result = runDrivetrainTuning(left, right, controller, {.1, 1}, config, {},
            [&] { if (pros::test_clock >= 15) controller.connected = false; return true; });
        check(result.capture == Result::cancelled);
        check(left.commands.front() != 0 && left.commands.back() == 0 && right.commands.back() == 0);
    }
    // Repositioning has a conservative default, an explicit limit, and respects
    // lower capture caps even when both arcade inputs saturate a wheel command.
    for (int scenario = 0; scenario < 5; ++scenario) {
        pros::test_clock = 0;
        pros::MotorGroup left, right;
        pros::Controller controller;
        controller.digital = [](pros::controller_digital_e_t b) { return b == pros::E_CONTROLLER_DIGITAL_X; };
        controller.analog = [](pros::controller_analog_e_t) { return -127; };
        auto settings = config;
        int expectedMv = 3000;
        if (scenario == 1) { settings.reposition_max_volts = 1; expectedMv = 1000; }
        if (scenario == 2) { settings.quasistatic.max_volts = 2; expectedMv = 2000; }
        if (scenario == 3) { settings.dynamic.max_volts = .5; expectedMv = 500; }
        if (scenario == 4) { settings.reposition_max_volts = 0; expectedMv = 0; }
        const auto result = runDrivetrainTuning(left, right, controller, {.1, 1}, settings, {},
                                               [] { return pros::test_clock < 15; });
        check(result.capture == Result::cancelled);
        check(left.commands.front() == -expectedMv);
        for (const auto* commands : {&left.commands, &right.commands}) {
            for (auto mv : *commands) check(std::abs(mv) <= expectedMv);
            check(commands->back() == 0);
        }
    }
    for (double invalid : {-1.0, 12.1, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        auto settings = config;
        settings.reposition_max_volts = invalid;
        check(!settings.valid());
        pros::MotorGroup left, right;
        pros::Controller controller;
        const auto result = runDrivetrainTuning(left, right, controller, {.1, 1}, settings);
        check(result.capture == Result::invalid_config);
        for (auto mv : left.commands) check(mv == 0);
        for (auto mv : right.commands) check(mv == 0);
    }
}
