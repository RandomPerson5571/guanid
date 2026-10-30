#pragma once

#include "guanid/guanid.hpp"

namespace guanid {

struct IgnoreState {
    void operator()(State) const {}
};

namespace detail {
template <typename Mechanism, typename Observer>
struct RoutineGuard {
    Mechanism& mechanism;
    Observer& observer;
    bool finished = false;

    Result finish(Result result) {
        const bool stopped = mechanism.setVoltage(0);
        observer(State::idle);
        finished = true;
        return stopped ? result : Result::hardware_error;
    }
    ~RoutineGuard() {
        if (!finished) {
            mechanism.setVoltage(0);
            observer(State::idle);
        }
    }
};
} // namespace detail

// Mechanism contract: uint32_t nowMs(), void sleepMs(uint32_t),
// bool setVoltage(int32_t millivolts), bool read(SideSample&, SideSample&).
// setVoltage must attempt BOTH sides even when one fails. Callbacks must be
// nonblocking; cleanup callbacks must not throw. The caller owns exclusivity.
template <typename Mechanism, typename KeepRunning, typename Sink, typename Observer = IgnoreState>
Result runRoutine(Mechanism& mechanism, Test test, Direction direction,
                  const Config& config, KeepRunning keep_running, Sink sink,
                  Observer observer = {}) {
    detail::RoutineGuard<Mechanism, Observer> guard{mechanism, observer};
    if (!config.valid() ||
        (test != Test::quasistatic && test != Test::dynamic) ||
        (direction != Direction::forward && direction != Direction::reverse))
        return guard.finish(Result::invalid_config);
    if (!mechanism.setVoltage(0)) return guard.finish(Result::hardware_error);
    observer(testState(test, direction));
    const std::uint32_t start = mechanism.nowMs();
    double active_volts = 0;
    for (;;) {
        if (!keep_running()) return guard.finish(Result::cancelled);
        Sample sample;
        sample.test = test;
        sample.direction = direction;
        sample.run_id = config.run_id;
        sample.commanded_volts = active_volts;
        sample.elapsed_ms = mechanism.nowMs() - start;
        if (sample.elapsed_ms >= config.duration_ms) return guard.finish(Result::completed);
        if (!mechanism.read(sample.left, sample.right) ||
            !sample.left.valid() || !sample.right.valid())
            return guard.finish(Result::hardware_error);
        // Do not log a late sample as if it were read at the previous deadline.
        if (mechanism.nowMs() - start >= config.duration_ms)
            return guard.finish(Result::completed);
        if (!sink(sample)) return guard.finish(Result::log_error);
        if (!keep_running()) return guard.finish(Result::cancelled);
        const auto elapsed = mechanism.nowMs() - start;
        if (elapsed >= config.duration_ms) return guard.finish(Result::completed);
        const auto millivolts = static_cast<std::int32_t>(std::lround(
            commandVolts(config, test, direction, elapsed) * 1000.0));
        if (!mechanism.setVoltage(millivolts)) return guard.finish(Result::hardware_error);
        active_volts = millivolts / 1000.0;
        // Schedule against elapsed time rather than adding callback latency to
        // every interval. Skip missed deadlines; never issue catch-up bursts.
        const auto now = mechanism.nowMs() - start;
        if (now >= config.duration_ms) return guard.finish(Result::completed);
        const auto next = std::min(config.duration_ms,
            (now / config.interval_ms + 1) * config.interval_ms);
        mechanism.sleepMs(next - now);
    }
}

} // namespace guanid
