#include "guanid/routine.hpp"

// Host-only example: exercises the real routine and CSV writer without motors.
// Each side follows a friction + velocity + acceleration model. This idealized
// simulation checks the workflow; it does not model V5 hardware or sensor noise.
namespace {
struct SimulatedSide {
    guanid::Feedforward gains;
    guanid::SideSample sample;

    void advance(double dt) {
        const double volts = sample.applied_volts;
        const double velocity = sample.velocity_mps;
        const double sign = velocity != 0 ? (velocity > 0 ? 1 : -1)
                                          : (volts > 0 ? 1 : -1);
        if (velocity == 0 && std::abs(volts) <= gains.kS) return;
        const double steady_velocity = (volts - gains.kS * sign) / gains.kV;
        const double tau = gains.kA / gains.kV;
        const double decay = std::exp(-dt / tau);
        const double next_velocity = steady_velocity + (velocity - steady_velocity) * decay;
        // The demo never reverses voltage while moving. Prevent numerical drift
        // through rest during coast-down between tests.
        if (velocity * next_velocity < 0) {
            const double stop_time = -tau * std::log(-steady_velocity / (velocity - steady_velocity));
            sample.position_m += steady_velocity * stop_time + tau * velocity;
            sample.velocity_mps = 0;
            return;
        }
        sample.position_m += steady_velocity * dt +
            (velocity - steady_velocity) * tau * (1 - decay);
        sample.velocity_mps = next_velocity;
    }
};

struct SimulatedMechanism {
    SimulatedSide left{{0.25, 2.1, 0.45}, {}};
    SimulatedSide right{{0.30, 2.4, 0.60}, {}};
    std::uint32_t clock_ms = 0;

    std::uint32_t nowMs() const { return clock_ms; }
    void sleepMs(std::uint32_t ms) {
        left.advance(ms / 1000.0);
        right.advance(ms / 1000.0);
        clock_ms += ms;
    }
    bool setVoltage(std::int32_t millivolts) {
        left.sample.applied_volts = right.sample.applied_volts = millivolts / 1000.0;
        return true;
    }
    bool read(guanid::SideSample& l, guanid::SideSample& r) const {
        l = left.sample;
        r = right.sample;
        return true;
    }
};
} // namespace

int main() {
    if (!guanid::writeCsvHeader(stdout)) return 1;
    guanid::Config config;
    config.duration_ms = 4000;
    config.ramp_volts_per_second = 1.0;
    SimulatedMechanism mechanism;
    for (const auto test : {guanid::Test::quasistatic, guanid::Test::dynamic}) {
        for (const auto direction : {guanid::Direction::forward, guanid::Direction::reverse}) {
            ++config.run_id;
            const auto result = guanid::runRoutine(mechanism, test, direction, config,
                [] { return true; },
                [](const guanid::Sample& sample) { return guanid::writeCsvSample(stdout, sample); });
            if (result != guanid::Result::completed) return 1;
            // Advance simulated time with zero voltage until both sides stop.
            mechanism.sleepMs(3000);
        }
    }
    return std::fflush(stdout) == 0 ? 0 : 1;
}
