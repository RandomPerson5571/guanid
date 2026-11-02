#include "guanid/pros.hpp"
#include <cstdlib>
#include <limits>
#include <type_traits>

using namespace guanid;
static_assert(!std::is_base_of<SysIdRoutine, ProsSysIdRoutine>::value);
static_assert(!std::is_constructible<ProsSysIdRoutine, ProsSysIdRoutine::Config,
              SysIdRoutine::Mechanism, ProsSysIdRoutine::Sink>::value);

static void check(bool ok) { if (!ok) std::abort(); }

static bool near(double a, double b) { return std::abs(a - b) < 1e-9; }

int main() {
    const Geometry geometry{0.1, 0.5};
    const double pi = 3.14159265358979323846;
    ProsSysIdRoutine::Config config;
    config.duration_ms = 100;
    config.step_volts = 7;
    const auto discard = [](const Record&) { return true; };

    // Public factories preserve motor gearing, degree offsets and physical SI capture.
    {
        pros::Motor wheel, arm;
        wheel.readings[0].degrees = 360;
        arm.readings[0].degrees = 360;
        std::vector<Record> rows;
        ProsSysIdRoutine routine(config, {
            ProsMotorChannel::linearInches("wheel", wheel, 4, 0.5),
            ProsMotorChannel::angularDegrees("arm", arm, 0.25, 30)
        }, [&](const Record& row) { rows.push_back(row); return true; });
        check(routine.startDynamic(Direction::forward));
        routine.update();
        check(near(rows[0].motor.position/metersPerInch, 2*pi));
        check(near(rows[1].motor.position/radiansPerDegree, 120));
        auto* csv = std::tmpfile();
        check(csv && writeRecordHeader(csv) && writePublicRecord(csv, rows[0]) && writePublicRecord(csv, rows[1]));
        std::rewind(csv);
        char buffer[1024]{};
        const auto length = std::fread(buffer, 1, sizeof(buffer)-1, csv);
        const std::string output(buffer, length);
        check(output.find(",wheel,in,") != std::string::npos);
        check(output.find(",arm,deg,") != std::string::npos);
        std::fclose(csv);
    }

    // Group averaging, wheel gearing, measured voltage, signs and channel names.
    {
        pros::MotorGroup left, right;
        left.readings.resize(2);
        left.readings[0].degrees = 360;
        left.readings[1].degrees = 720;
        left.readings[0].rpm = 60;
        left.readings[1].rpm = 120;
        std::vector<Record> rows;
        ProsSysIdRoutine routine(config, {
            ProsMotorChannel::linear("left", left, geometry, -1),
            ProsMotorChannel::linear("right", right, geometry)
        }, [&](const Record& row) { rows.push_back(row); return true; });
        check(routine.startDynamic(Direction::forward));
        // Delivered voltage is read, rather than synthesized from the command.
        left.readings[0].millivolts = -2000;
        left.readings[1].millivolts = -4000;
        check(routine.update() == Result::running);
        check(rows.size() == 2 && rows[0].motor.name == "left" && rows[1].motor.name == "right");
        check(rows[0].motor.units == Units::linear);
        check(near(rows[0].motor.volts, -3));
        check(near(rows[0].motor.position, 1.5 * pi * 0.1 * 0.5));
        check(near(rows[0].motor.velocity, 1.5 * pi * 0.1 * 0.5));
        check(left.commands.back() == -7000 && right.commands.back() == 7000);
        check(routine.cancel() == Result::cancelled);
        check(left.commands.back() == 0 && right.commands.back() == 0);
    }
    // A single PROS motor, radians, gearing, offset and destructor cleanup.
    {
        pros::Motor motor;
        motor.readings[0].degrees = 360;
        motor.readings[0].rpm = 60;
        std::vector<Record> rows;
        {
            ProsSysIdRoutine routine(config, {
                ProsMotorChannel::angular("arm", motor, 0.25, 0.2)
            }, [&](const Record& row) { rows.push_back(row); return true; });
            check(routine.startDynamic(Direction::reverse));
            routine.update();
            check(rows[0].motor.units == Units::angular);
            check(near(rows[0].motor.position, pi / 2 + 0.2));
            check(near(rows[0].motor.velocity, pi / 2));
            check(motor.commands.back() == -7000);
        }
        check(motor.commands.back() == 0);
    }
    // Blocking tests and timeout retain the portable engine's lifecycle.
    {
        pros::Motor motor;
        ProsSysIdRoutine routine(config, {ProsMotorChannel::angular("wheel", motor)}, discard);
        check(routine.quasistatic(Direction::forward, [] { return true; }) == Result::completed);
        check(!routine.running() && motor.commands.back() == 0);
        check(routine.state() == State::idle);
    }
    // Command failure must still attempt every channel and then stop all of them.
    {
        pros::Motor left, right;
        left.fail_drive = true;
        ProsSysIdRoutine routine(config, {
            ProsMotorChannel::angular("left", left), ProsMotorChannel::angular("right", right)
        }, discard);
        check(routine.startDynamic(Direction::forward));
        check(routine.update() == Result::hardware_error);
        check(right.commands == std::vector<std::int32_t>({0, 7000, 0}));
        check(left.commands.back() == 0);
    }
    // Bad encoder units, failed telemetry and sink failure all stop both sides.
    for (int failure = 0; failure < 3; ++failure) {
        pros::MotorGroup left, right;
        if (failure == 0) left.readings[0].units = pros::MotorUnits::rotations;
        if (failure == 1) left.readings[0].degrees = std::numeric_limits<double>::infinity();
        ProsSysIdRoutine routine(config, {
            ProsMotorChannel::linear("left", left, geometry),
            ProsMotorChannel::linear("right", right, geometry)
        }, [failure](const Record&) { return failure != 2; });
        check(routine.startDynamic(Direction::forward));
        check(routine.update() == (failure == 2 ? Result::log_error : Result::hardware_error));
        check(left.commands.back() == 0 && right.commands.back() == 0);
    }
    // Invalid channel configurations send only stop commands.
    for (int invalid = 0; invalid < 8; ++invalid) {
        pros::Motor left, right;
        std::vector<ProsMotorChannel> channels;
        switch (invalid) {
        case 0: break; // empty list
        case 1: channels = {ProsMotorChannel::linear("bad name", left, geometry)}; break;
        case 2: channels = {ProsMotorChannel::linear("left", left, {0, 1})}; break;
        case 3: channels = {ProsMotorChannel::linear("left", left, geometry, 0)}; break;
        case 4: channels = {ProsMotorChannel::angular("left", left, 1,
                               std::numeric_limits<double>::infinity())}; break;
        case 5: channels = {ProsMotorChannel::angular("same", left),
                           ProsMotorChannel::angular("same", right)}; break;
        case 6: channels = {ProsMotorChannel::angular("left", left),
                           ProsMotorChannel::angular("again", left)}; break;
        case 7:
            left.readings.clear();
            channels = {ProsMotorChannel::angular("empty", left)};
            break;
        }
        ProsSysIdRoutine routine(config, channels, discard);
        check(!routine.startDynamic(Direction::forward));
        check(routine.result() == Result::invalid_config);
        for (auto mv : left.commands) check(mv == 0);
        for (auto mv : right.commands) check(mv == 0);
    }
    // A failed stop takes precedence over cancellation and still stops peers.
    {
        pros::Motor left, right;
        ProsSysIdRoutine routine(config, {
            ProsMotorChannel::angular("left", left), ProsMotorChannel::angular("right", right)
        }, discard);
        check(routine.startDynamic(Direction::forward));
        left.fail_stop = true;
        check(routine.update(false) == Result::hardware_error);
        check(left.commands.back() == 0 && right.commands.back() == 0);
    }
}
