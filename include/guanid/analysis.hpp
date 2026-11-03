#pragma once

#include "guanid/sysid.hpp"
#include <array>
#include <limits>
#include <map>
#include <set>
#include <tuple>

namespace guanid {

// All thresholds here use SI, even when analyzePublic converts the final gains.
// Windows count samples; trims and gap limits use seconds. Derivative windows
// must be odd and at least three samples to fit a local quadratic velocity curve.
struct AnalysisConfig {
    Model model = Model::simple;
    std::size_t median_window = 5, derivative_window = 5;
    double min_velocity = 0.02, trim_start_s = 0.05, trim_end_s = 0, max_gap_s = 0.1;
    bool require_four_tests = true;
    bool valid() const {
        return (model == Model::simple || model == Model::elevator || model == Model::arm) &&
            median_window > 0 && median_window % 2 == 1 && derivative_window >= 3 && derivative_window % 2 == 1 &&
            std::isfinite(min_velocity) && min_velocity >= 0 &&
            std::isfinite(trim_start_s) && trim_start_s >= 0 &&
            std::isfinite(trim_end_s) && trim_end_s >= 0 && std::isfinite(max_gap_s) && max_gap_s > 0;
    }
};
// Voltage-output model: kS/kG are volts, kV multiplies m/s or rad/s, and
// kA multiplies m/s^2 or rad/s^2. Arm position is radians from horizontal.
struct Gains {
    double kS = 0, kV = 0, kA = 0, kG = 0;
    double calculate(Model model, double position, double velocity, double acceleration) const {
        return SimpleMotorFeedforward{kS,kV,kA}.calculate(velocity, acceleration) +
            (model == Model::arm ? kG * std::cos(position) : model == Model::elevator ? kG : 0);
    }
};
struct Diagnostic {
    Record record;
    double velocity = 0, acceleration = 0, predicted_voltage = 0, residual = 0;
};
struct Fit {
    bool ok = false;
    std::string error;
    Gains gains;
    std::size_t samples = 0;
    // NaN means R² is undefined (constant dependent variable).
    double acceleration_r_squared = std::numeric_limits<double>::quiet_NaN();
    double voltage_r_squared = std::numeric_limits<double>::quiet_NaN();
    double voltage_rmse = 0, excitation_ratio = 0;
};
struct Analysis : Fit {
    Units units = Units::linear;
    Model model = Model::simple;
    Fit forward, reverse;
    std::vector<std::string> warnings;
    std::vector<Diagnostic> diagnostics;
};

namespace detail {
// Scaled, twice-orthogonalized QR for small dense least-squares problems.
// No normal equations (which square the condition number), external libraries,
// exceptions or variable-length arrays. Reject nearly dependent regressors.
inline bool leastSquares(const std::vector<std::array<double,4>>& rows,
                         const std::vector<double>& y, std::size_t n,
                         std::array<double,4>& solution, double& excitation) {
    if (rows.size() < n || rows.size() != y.size()) return false;
    std::array<std::vector<double>,4> q;
    double r[4][4] = {}, scale[4] = {}, qty[4] = {};
    excitation = 1;
    for (std::size_t j = 0; j < n; ++j) {
        q[j].resize(rows.size());
        for (std::size_t i = 0; i < rows.size(); ++i) scale[j] = std::hypot(scale[j], rows[i][j]);
        if (!(scale[j] > 0) || !std::isfinite(scale[j])) return false;
        for (std::size_t i = 0; i < rows.size(); ++i) q[j][i] = rows[i][j] / scale[j];
        for (int pass = 0; pass < 2; ++pass) {
            for (std::size_t k = 0; k < j; ++k) {
                double dot = 0;
                for (std::size_t i = 0; i < rows.size(); ++i) dot += q[k][i] * q[j][i];
                r[k][j] += dot;
                for (std::size_t i = 0; i < rows.size(); ++i) q[j][i] -= dot * q[k][i];
            }
        }
        for (double v : q[j]) r[j][j] = std::hypot(r[j][j], v);
        excitation = std::min(excitation, r[j][j]);
        if (r[j][j] < 1e-6) return false;
        for (std::size_t i = 0; i < rows.size(); ++i) {
            q[j][i] /= r[j][j];
            qty[j] += q[j][i] * y[i];
        }
    }
    std::array<double,4> scaled{};
    for (int j = static_cast<int>(n) - 1; j >= 0; --j) {
        double v = qty[j];
        for (std::size_t k = j + 1; k < n; ++k) v -= r[j][k] * scaled[k];
        scaled[j] = v / r[j][j];
        solution[j] = scaled[j] / scale[j];
        if (!std::isfinite(solution[j])) return false;
    }
    return true;
}
inline Fit fit(const std::vector<Diagnostic>& points, Model model, int direction = 0) {
    Fit out;
    std::vector<std::array<double,4>> x;
    std::vector<double> y;
    for (const auto& p : points) {
        if (direction && (p.record.direction == Direction::forward ? 1 : -1) != direction) continue;
        const double gravity = model == Model::arm ? std::cos(p.record.motor.position) : 1;
        x.push_back({p.record.motor.volts, p.velocity, p.velocity > 0 ? 1.0 : -1.0, gravity});
        y.push_back(p.acceleration);
    }
    out.samples = x.size();
    if (x.size() < 12) { out.error = "Need at least 12 moving samples"; return out; }
    std::array<double,4> b{};
    if (!leastSquares(x, y, model == Model::simple ? 3 : 4, b, out.excitation_ratio)) {
        out.error = "Unexcited or rank-deficient data; collect ramps, transients and both directions"; return out;
    }
    if (b[0] <= 0) { out.error = "Nonpositive voltage-to-acceleration gain; check sensor signs and timing"; return out; }
    // Regression fits a = b0*V + b1*v + b2*sign(v) + b3*gravity.
    // Rearrange into the voltage model, rather than fitting noisy acceleration
    // as an independent regressor. Simple models omit the gravity column.
    out.gains = {-b[2]/b[0], -b[1]/b[0], 1/b[0], model == Model::simple ? 0 : -b[3]/b[0]};
    if (!std::isfinite(out.gains.kS) || !std::isfinite(out.gains.kV) ||
        !std::isfinite(out.gains.kA) || !std::isfinite(out.gains.kG)) {
        out.error = "Nonfinite fitted gains"; return out;
    }
    double mean_a = 0, mean_u = 0;
    for (std::size_t i = 0; i < y.size(); ++i) { mean_a += y[i]/y.size(); mean_u += x[i][0]/y.size(); }
    double sse_a = 0, sst_a = 0, sse_u = 0, sst_u = 0;
    for (std::size_t i = 0; i < y.size(); ++i) {
        double predicted_a = 0;
        for (std::size_t j = 0; j < (model == Model::simple ? 3u : 4u); ++j) predicted_a += x[i][j]*b[j];
        sse_a += std::pow(y[i]-predicted_a,2); sst_a += std::pow(y[i]-mean_a,2);
        sse_u += std::pow((y[i]-predicted_a)/b[0],2); sst_u += std::pow(x[i][0]-mean_u,2);
    }
    out.voltage_rmse = std::sqrt(sse_u/y.size());
    if (!std::isfinite(out.voltage_rmse) || !std::isfinite(sse_a) ||
        !std::isfinite(sst_a) || !std::isfinite(sst_u)) {
        out.error = "Nonfinite fit diagnostics"; return out;
    }
    if (sst_a > 1e-20) out.acceleration_r_squared = 1-sse_a/sst_a;
    if (sst_u > 1e-20) out.voltage_r_squared = 1-sse_u/sst_u;
    out.ok = true;
    return out;
}
} // namespace detail

// Analyze exactly one named channel. Every run is differentiated independently.
// Position/velocity are metres or radians; arm angles must be radians from horizontal.
inline Analysis analyze(const std::vector<Record>& records, const std::string& motor,
                        const AnalysisConfig& config = {}) {
    Analysis out;
    out.model = config.model;
    auto fail = [&](const std::string& error) { out.error = error; return out; };
    if (!config.valid()) return fail("Invalid analysis configuration");
    // Never filter or differentiate across independent tests. A repeated test
    // needs a new run ID; otherwise its reset timestamp would break ordering.
    using Key = std::tuple<std::uint32_t,Test,Direction>;
    std::map<Key,std::vector<Record>> runs;
    bool first = true;
    for (const auto& row : records) {
        if (row.motor.name != motor) continue;
        if (!row.motor.valid() || !std::isfinite(row.time_s) || row.time_s < 0 ||
            (row.test != Test::quasistatic && row.test != Test::dynamic) ||
            (row.direction != Direction::forward && row.direction != Direction::reverse)) return fail("Invalid sample");
        if (first) { out.units = row.motor.units; first = false; }
        if (out.units != row.motor.units) return fail("Mixed units in channel");
        auto& run = runs[{row.run_id,row.test,row.direction}];
        if (!run.empty() && row.time_s <= run.back().time_s) return fail("Nonincreasing time; use unique run IDs");
        run.push_back(row);
    }
    if (first) return fail("No samples for channel");
    if (config.model == Model::arm && out.units != Units::angular) return fail("Arm requires radians");
    std::set<std::pair<Test,Direction>> coverage;
    std::size_t mismatches = 0;
    const auto radius = config.median_window/2, half = config.derivative_window/2;
    for (const auto& entry : runs) {
        const auto& run = entry.second;
        // Bound before adding unsigned window sizes (also handles huge options).
        if (radius >= run.size()/2 || half >= run.size()/2 || radius + half >= run.size()/2) continue;
        std::vector<double> velocity;
        for (const auto& row : run) velocity.push_back(row.motor.velocity);
        for (std::size_t i = radius; i + radius < run.size(); ++i) {
            std::vector<double> window;
            for (std::size_t j = i-radius; j <= i+radius; ++j) window.push_back(run[j].motor.velocity);
            std::nth_element(window.begin(),window.begin()+radius,window.end());
            velocity[i] = window[radius];
        }
        for (std::size_t i = half+radius; i + half+radius < run.size(); ++i) {
            if (run[i].time_s-run.front().time_s < config.trim_start_s ||
                run.back().time_s-run[i].time_s < config.trim_end_s) continue;
            // Reject the entire support window if it crosses rest, a direction
            // change, or a sampling gap. Checking raw samples also prevents the
            // median filter from concealing an invalid differentiation interval.
            bool valid = std::abs(velocity[i]) > config.min_velocity;
            for (std::size_t j = i-half-radius; j <= i+half+radius; ++j) {
                if (std::abs(run[j].motor.velocity) <= config.min_velocity ||
                    (run[j].motor.velocity > 0) != (velocity[i] > 0) ||
                    (j > i-half-radius && run[j].time_s-run[j-1].time_s > config.max_gap_s)) valid = false;
            }
            if (!valid) continue;
            std::vector<std::array<double,4>> x;
            std::vector<double> y;
            for (std::size_t j = i-half; j <= i+half; ++j) {
                const double t = run[j].time_s-run[i].time_s;
                x.push_back({1,t,t*t,0}); y.push_back(velocity[j]);
            }
            // Center time on this sample: in v(t)=c0+c1*t+c2*t^2, c1 is
            // acceleration at the center, including for irregular sample times.
            std::array<double,4> polynomial{};
            double excitation;
            if (!detail::leastSquares(x,y,3,polynomial,excitation)) continue;
            out.diagnostics.push_back({run[i],velocity[i],polynomial[1],0,0});
            coverage.insert({run[i].test,run[i].direction});
            if ((velocity[i] > 0) != (run[i].direction == Direction::forward)) ++mismatches;
        }
    }
    // Coverage counts retained moving samples, not merely tests present in CSV.
    // A captured profile entirely removed by trimming cannot identify the model.
    if (coverage.size() < 4) {
        if (config.require_four_tests) return fail("Missing usable tests; capture quasistatic/dynamic in both directions");
        out.warnings.push_back("Incomplete four-test coverage");
    }
    static_cast<Fit&>(out) = detail::fit(out.diagnostics,config.model);
    if (!out.ok) return out;
    out.forward = detail::fit(out.diagnostics,config.model,1);
    out.reverse = detail::fit(out.diagnostics,config.model,-1);
    if (out.gains.kS < 0 || out.gains.kV <= 0) out.warnings.push_back("Nonphysical friction/velocity gains");
    if (!std::isfinite(out.acceleration_r_squared) || out.acceleration_r_squared < 0.8)
        out.warnings.push_back("Weak acceleration fit; inspect residuals and sensor timing");
    if (mismatches > out.samples/10) out.warnings.push_back("Motion disagrees with test direction");
    if (out.forward.ok && out.reverse.ok) {
        const auto& f = out.forward.gains; const auto& r = out.reverse.gains;
        auto differs = [](double a,double b) { return std::abs(a-b) > 0.25*std::max({std::abs(a),std::abs(b),1e-6}); };
        if (differs(f.kS,r.kS) || differs(f.kV,r.kV) || differs(f.kA,r.kA))
            out.warnings.push_back("Forward/reverse gains differ by more than 25 percent");
    } else out.warnings.push_back("Independent direction fits unavailable (elevator gravity and friction require both directions)");
    for (auto& p : out.diagnostics) {
        p.predicted_voltage = out.gains.calculate(config.model,p.record.motor.position,p.velocity,p.acceleration);
        p.residual = p.record.motor.volts-p.predicted_voltage;
    }
    return out;
}

struct FeedbackGains { bool valid = false; double kP = 0, kI = 0, kD = 0; };
// Continuous-time velocity P controller: desired time constant tau, output volts.
// Assumes feedforward is applied, no delay/saturation, and kA > 0. Validate on hardware.
inline FeedbackGains suggestVelocityFeedback(const Gains& gains, double tau_s) {
    const double p = gains.kA/tau_s-gains.kV;
    if (!std::isfinite(tau_s) || tau_s <= 0 || !std::isfinite(p) || gains.kA <= 0 || gains.kV <= 0 || p < 0) return {};
    return {true,p,0,0};
}
// Position PD pole placement: a + 2*zeta*omega*v + omega²*x = 0.
inline FeedbackGains suggestPositionFeedback(const Gains& gains, double omega_radps, double damping = 1) {
    const double p = gains.kA*omega_radps*omega_radps, d = 2*damping*omega_radps*gains.kA-gains.kV;
    if (!std::isfinite(p) || !std::isfinite(d) || !std::isfinite(damping) || omega_radps <= 0 || damping <= 0 ||
        gains.kA <= 0 || gains.kV <= 0 || d < 0) return {};
    return {true,p,0,d};
}

// Derive a combined linear or differential angular channel from paired samples.
// Angular: omega=(right-left)/track_width, U=(right-left)/2, so gains use
// differential volts per rad/s. Turn in place with opposite wheel commands.
inline bool deriveDrivetrain(const std::vector<Record>& records, const std::string& left,
                             const std::string& right, const std::string& name,
                             std::vector<Record>& derived, double track_width_m = 0) {
    if (left == right || name == left || name == right || !std::isfinite(track_width_m) || track_width_m < 0) return false;
    using Key = std::tuple<std::uint32_t,Test,Direction,double>;
    std::map<Key,Record> l, r;
    for (const auto& row : records) {
        if (row.motor.name == name) return false;
        if (row.motor.name != left && row.motor.name != right) continue;
        if (!row.motor.valid() || row.motor.units != Units::linear || !std::isfinite(row.time_s)) return false;
        auto& side = row.motor.name == left ? l : r;
        if (!side.emplace(Key{row.run_id,row.test,row.direction,row.time_s},row).second) return false;
    }
    if (l.empty() || l.size() != r.size()) return false;
    std::vector<Record> output;
    for (const auto& entry : l) {
        auto it = r.find(entry.first); if (it == r.end()) return false;
        Record row = entry.second;
        const auto& a = row.motor; const auto& b = it->second.motor;
        MotorSample sample; sample.name = name;
        if (track_width_m > 0) sample.voltage((b.volts-a.volts)/2)
            .angularPosition((b.position-a.position)/track_width_m)
            .angularVelocity((b.velocity-a.velocity)/track_width_m);
        else sample.voltage((a.volts+b.volts)/2).linearPosition((a.position+b.position)/2)
            .linearVelocity((a.velocity+b.velocity)/2);
        if (!sample.valid()) return false;
        row.motor = sample; output.push_back(row);
    }
    derived.insert(derived.end(),output.begin(),output.end());
    return true;
}

} // namespace guanid
