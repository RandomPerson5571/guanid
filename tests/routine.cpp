#include "guanid/routine.hpp"
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <vector>

void check(bool condition) { if (!condition) std::abort(); }

struct FakeMechanism {
    std::uint32_t clock = 0;
    std::uint32_t read_delay = 0;
    bool read_ok = true;
    bool command_ok = true;
    bool stop_ok = true;
    bool nonfinite = false;
    bool fail_final_stop = false;
    int stops = 0;
    std::vector<std::int32_t> commands;
    std::vector<std::uint32_t> times;

    std::uint32_t nowMs() { return clock; }
    void sleepMs(std::uint32_t ms) { clock += ms; }
    bool setVoltage(std::int32_t value) {
        commands.push_back(value);
        times.push_back(clock);
        if (value == 0 && ++stops > 1 && fail_final_stop) return false;
        return value == 0 ? stop_ok : command_ok;
    }
    bool read(guanid::SideSample& left, guanid::SideSample& right) {
        clock += read_delay;
        left = {commands.back() / 1000.0, 1, 0.5};
        right = left;
        if (nonfinite) right.velocity_mps = std::numeric_limits<double>::infinity();
        return read_ok;
    }
};

int main() {
    using namespace guanid;
    Config config;
    config.duration_ms = 100;
    config.run_id = 42;
    auto enabled = [] { return true; };
    auto sink = [](const Sample&) { return true; };
    FakeMechanism mechanism;
    std::vector<Sample> samples;
    std::vector<State> states;
    auto result = runRoutine(mechanism, Test::dynamic, Direction::reverse, config,
        enabled, [&](const Sample& sample) { samples.push_back(sample); return true; },
        [&](State state) { states.push_back(state); });
    check(result == Result::completed && mechanism.clock == 100);
    check(mechanism.commands.front() == 0 && mechanism.commands.back() == 0);
    check(samples.size() == 5 && samples[0].commanded_volts == 0);
    check(samples[1].elapsed_ms == 20 && samples[1].commanded_volts == -3);
    check(samples[1].left.applied_volts == -3 && samples[1].run_id == 42);
    check(states == std::vector<State>({State::dynamic_reverse, State::idle}));

    mechanism = {};
    result = runRoutine(mechanism, Test::dynamic, Direction::forward, config,
        [&] { return mechanism.clock < 40; }, sink);
    check(result == Result::cancelled && mechanism.commands.back() == 0);
    mechanism = {};
    result = runRoutine(mechanism, Test::dynamic, Direction::forward, config, enabled,
        [](const Sample&) { return false; });
    check(result == Result::log_error && mechanism.commands.back() == 0);
    mechanism = {};
    mechanism.read_ok = false;
    check(runRoutine(mechanism, Test::dynamic, Direction::forward, config, enabled, sink) == Result::hardware_error);
    check(mechanism.commands.back() == 0);
    mechanism = {};
    mechanism.command_ok = false;
    check(runRoutine(mechanism, Test::dynamic, Direction::forward, config, enabled, sink) == Result::hardware_error);
    check(mechanism.commands.back() == 0);
    mechanism = {};
    mechanism.nonfinite = true;
    check(runRoutine(mechanism, Test::dynamic, Direction::forward, config, enabled, sink) == Result::hardware_error);
    mechanism = {};
    mechanism.stop_ok = false;
    check(runRoutine(mechanism, Test::dynamic, Direction::forward, config, enabled, sink) == Result::hardware_error);
    check(mechanism.commands.size() == 2); // Initial zero and cleanup retry.
    mechanism = {};
    mechanism.fail_final_stop = true;
    check(runRoutine(mechanism, Test::dynamic, Direction::forward, config, enabled, sink) == Result::hardware_error);
    check(mechanism.clock == 100 && mechanism.commands.back() == 0);
    mechanism = {};
    states.clear();
    try {
        runRoutine(mechanism, Test::dynamic, Direction::forward, config, enabled,
            [](const Sample&) -> bool { throw std::runtime_error("sink"); },
            [&](State state) { states.push_back(state); });
        check(false);
    } catch (const std::runtime_error&) {
        check(mechanism.commands.back() == 0 && states.back() == State::idle);
    }
    mechanism = {};
    config.interval_ms = 0;
    check(runRoutine(mechanism, Test::dynamic, Direction::forward, config, enabled, sink) == Result::invalid_config);
    check(mechanism.commands == std::vector<std::int32_t>({0}));
    config.interval_ms = 20;

    // Rollover and missed deadlines use real elapsed time, with no catch-up bursts.
    mechanism = {};
    mechanism.clock = std::numeric_limits<std::uint32_t>::max() - 30;
    const auto start = mechanism.clock;
    check(runRoutine(mechanism, Test::dynamic, Direction::forward, config, enabled, sink) == Result::completed);
    check(mechanism.clock - start == 100);
    mechanism = {};
    mechanism.read_delay = 25;
    check(runRoutine(mechanism, Test::dynamic, Direction::forward, config, enabled, sink) == Result::completed);
    check(mechanism.times[1] == 25 && mechanism.times[2] == 65);
    mechanism = {};
    check(runRoutine(mechanism, Test::dynamic, Direction::forward, config, enabled,
        [&](const Sample&) { mechanism.clock += 120; return true; }) == Result::completed);
    check(mechanism.commands == std::vector<std::int32_t>({0, 0}));
}
