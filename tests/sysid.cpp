#include "guanid/sysid.hpp"
#include <cstdlib>
#include <limits>
#include <stdexcept>

void check(bool value) { if (!value) std::abort(); }
int main() {
    using namespace guanid;
    std::uint32_t clock = std::numeric_limits<std::uint32_t>::max()-30;
    double volts = 0;
    std::vector<double> commands;
    std::vector<Record> rows;
    std::vector<State> states;
    SysIdRoutine::Config config;
    check(config.duration_ms == 10000 && config.step_volts == 7 && config.ramp_volts_per_second == 1);
    config.duration_ms = 100;
    bool read_ok = true, sink_ok = true, throw_log = false, stop_ok = true;
    SysIdRoutine routine(config,{
        [&](double v) { volts=v; commands.push_back(v); return v != 0 || stop_ok; },
        [&](SysIdLog& log) {
            if (throw_log) throw std::runtime_error("sensor");
            log.motor("flywheel").voltage(volts).angularPosition(clock*0.01).angularVelocity(volts);
            return read_ok;
        }}, { [&] { return clock; }, [&](std::uint32_t ms) { clock+=ms; } },
        [&](const Record& r) { rows.push_back(r); return sink_ok; },
        [&](State s) { states.push_back(s); });
    check(routine.dynamic(Direction::reverse,[] { return true; }) == Result::completed);
    check(rows.size() == 5 && rows[0].motor.volts == 0 && rows[1].motor.volts == -7);
    check(rows[1].time_s == 0.02 && rows[1].motor.units == Units::angular && volts == 0);
    check(states.front() == State::dynamic_reverse && states.back() == State::idle);
    const auto first_id = rows.front().run_id;
    rows.clear();
    check(routine.quasistatic(Direction::forward,[] { return true; }) == Result::completed);
    check(rows.front().run_id != first_id && rows[2].motor.volts == 0.02);
    check(routine.startDynamic(Direction::forward));
    check(routine.update() == Result::running);
    check(routine.update(false) == Result::cancelled && volts == 0);
    read_ok = false;
    check(routine.dynamic(Direction::forward,[] { return true; }) == Result::hardware_error && volts == 0);
    read_ok = true; sink_ok = false;
    check(routine.dynamic(Direction::forward,[] { return true; }) == Result::log_error && volts == 0);
    sink_ok = true;
    check(routine.startDynamic(Direction::forward));
    routine.update(); clock += 20; throw_log = true;
    try { routine.update(); check(false); } catch (const std::runtime_error&) {}
    check(!routine.running() && volts == 0);
    throw_log = false;
    stop_ok = false;
    check(!routine.startDynamic(Direction::forward) && routine.result() == Result::hardware_error);
    stop_ok = true;
    check(routine.startDynamic(Direction::forward)); routine.update();
    check(!routine.startDynamic(Direction::reverse) && volts == 0);

    MotorSample mixed;
    mixed.name="arm";
    mixed.voltage(1).linearPosition(2).angularVelocity(3);
    check(!mixed.valid());
    SysIdLog log;
    log.motor("bad,name").voltage(1).linearPosition(1).linearVelocity(1);
    check(!log.valid());
    const ElevatorFeedforward elevator(0.2,2,0.5,0.7);
    const ArmFeedforward arm(0.2,2,0.5,0.7);
    check(std::abs(elevator.calculate(-1,-2)+2.5) < 1e-12);
    check(std::abs(arm.calculate(0,1,2)-3.9) < 1e-12);
    check(std::abs(arm.calculate(3.141592653589793/2,0)) < 1e-12);
    auto* file = std::tmpfile();
    check(file && writeRecordHeader(file) && writeRecord(file,rows[1]));
    std::fclose(file);
}
