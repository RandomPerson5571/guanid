#include "guanid/analysis.hpp"
#include <iostream>

// Entire SysId workflow in C++, including in-memory analysis. Simulated flywheel
// with radians and seconds; replace just the drive/log/runtime callbacks on PROS.
int main() {
    using namespace guanid;
    std::uint32_t clock = 0;
    double volts = 0, position = 0, velocity = 0;
    const Gains truth{0.25,2.1,0.45,0};
    std::vector<Record> capture;
    SysIdRoutine::Config config;
    config.duration_ms=4000; config.step_volts=3;
    if (!writeRecordHeader(stdout)) return 1;
    SysIdRoutine routine(config, {
        [&](double command) { volts=command; return true; },
        [&](SysIdLog& log) {
            log.motor("flywheel").voltage(volts).angularPosition(position).angularVelocity(velocity);
            return true;
        }}, {
        [&] { return clock; },
        [&](std::uint32_t ms) {
            // Fine-step integration includes a static-friction dead zone.
            for (std::uint32_t i=0; i<ms; ++i) {
                if (std::abs(velocity)<1e-9 && std::abs(volts)<=truth.kS) { velocity=0; continue; }
                const double sign=velocity != 0 ? (velocity>0 ? 1 : -1) : (volts>0 ? 1 : -1);
                const double a=(volts-truth.kS*sign-truth.kV*velocity)/truth.kA;
                const double next=velocity+a*0.001;
                velocity=next*velocity<0 && volts==0 ? 0 : next;
                position+=velocity*0.001;
            }
            clock+=ms;
        }},
        [&](const Record& row) { capture.push_back(row); return writeRecord(stdout,row); });
    for (auto direction : {Direction::forward,Direction::reverse}) {
        velocity=0;
        if (routine.quasistatic(direction,[] { return true; }) != Result::completed) return 1;
        velocity=0;
        if (routine.dynamic(direction,[] { return true; }) != Result::completed) return 1;
    }
    const auto fit=analyze(capture,"flywheel");
    if (!fit.ok) { std::cerr << fit.error << '\n'; return 1; }
    std::cerr << "flywheel: kS=" << fit.gains.kS << " kV=" << fit.gains.kV << " kA=" << fit.gains.kA << '\n';
    if (std::abs(fit.gains.kS-truth.kS)>0.02 || std::abs(fit.gains.kV-truth.kV)>0.02 ||
        std::abs(fit.gains.kA-truth.kA)>0.02) return 1;
    const SimpleMotorFeedforward feedforward{fit.gains.kS,fit.gains.kV,fit.gains.kA};
    (void)feedforward.calculate(1.0,0.5); // volts for target rad/s and rad/s²
    return std::fflush(stdout) == 0 ? 0 : 1;
}
