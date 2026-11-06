#pragma once

// Configurable robot-facing unit boundary. Regression and legacy APIs stay SI.
#include "guanid/analysis.hpp"

namespace guanid {
inline constexpr double metersPerInch = 0.0254;
inline constexpr double radiansPerDegree = 3.14159265358979323846 / 180;
enum class LinearUnit { meters, inches };
enum class AngularUnit { radians, degrees };
struct OutputUnits {
    LinearUnit linear = LinearUnit::inches;
    AngularUnit angular = AngularUnit::degrees;
    bool valid() const {
        return (linear == LinearUnit::meters || linear == LinearUnit::inches) &&
               (angular == AngularUnit::radians || angular == AngularUnit::degrees);
    }
    double scale(Units dimension) const {
        return dimension == Units::linear ? (linear == LinearUnit::inches ? metersPerInch : 1)
                                         : (angular == AngularUnit::degrees ? radiansPerDegree : 1);
    }
    const char* name(Units dimension) const {
        return dimension == Units::linear ? (linear == LinearUnit::inches ? "in" : "m")
                                         : (angular == AngularUnit::degrees ? "deg" : "rad");
    }
};
inline double siPerPublicUnit(Units dimension) { return OutputUnits{}.scale(dimension); }
inline Gains gainsInUnits(Gains gainsSI, Units dimension, OutputUnits units = {}) {
    gainsSI.kV *= units.scale(dimension);
    gainsSI.kA *= units.scale(dimension);
    return gainsSI;
}
// Input/output gains may use different units. Width uses the input linear unit.
// For an ideal in-place differential-drive turn, wheel speed = omega * width/2.
// kS stays volts. This estimates turning gains; scrub requires a measured spin fit.
inline Gains lateralToAngular(Gains lateral, double trackWidth,
                              OutputUnits units = {}) {
    if (!units.valid() || !std::isfinite(trackWidth) || trackWidth <= 0 ||
        !std::isfinite(lateral.kS) || !std::isfinite(lateral.kV) || !std::isfinite(lateral.kA)) {
        const auto nan = std::numeric_limits<double>::quiet_NaN();
        return {nan, nan, nan, nan};
    }
    const double radius = trackWidth * units.scale(Units::angular) / 2;
    return {lateral.kS, lateral.kV * radius, lateral.kA * radius, 0};
}
// Public velocities are numerically larger than their SI equivalents, so the
// corresponding voltage-per-velocity and voltage-per-acceleration gains shrink.
// kS and kG are already volts and need no scaling.
inline Gains gainsInPublicUnits(Gains gains, Units dimension) {
    const double scale = siPerPublicUnit(dimension);
    gains.kV *= scale;
    gains.kA *= scale;
    return gains;
}

struct PublicAnalysis {
    Analysis si; // Explicit compatibility access to the original SI diagnostics.
    OutputUnits output;
    const char* positionUnit() const { return output.name(si.units); }
    Gains gains() const { return gainsInUnits(si.gains, si.units, output); }
    // Convert only the arm's angle back to radians for cos(). Converted kV/kA
    // already expect public velocity/acceleration; scaling those inputs again
    // would apply the unit conversion twice. Check si.ok before using the result.
    double calculate(double position, double velocity, double acceleration) const {
        const auto g = gains();
        return g.calculate(si.model, si.units == Units::angular ? position * output.scale(Units::angular) : position,
                           velocity, acceleration);
    }
};

// Records/config are explicitly SI, preserving the fitter and thresholds.
// Output gains and calculate() use selected units (inches/degrees by default).
inline PublicAnalysis analyzePublic(const std::vector<Record>& records, const std::string& motor,
                                    const AnalysisConfig& configSI = {}, OutputUnits output = {}) {
    return {analyze(records, motor, configSI), output};
}

// Same named CSV schema, with explicit unit tokens; legacy writeRecord stays SI.
inline bool writePublicRecord(std::FILE* file, const Record& row, OutputUnits output = {}) {
    if (!output.valid() || !file || !row.motor.valid() || !std::isfinite(row.time_s) || row.time_s < 0 ||
        (row.test != Test::quasistatic && row.test != Test::dynamic) ||
        (row.direction != Direction::forward && row.direction != Direction::reverse)) return false;
    const auto& sample = row.motor;
    const double scale = output.scale(sample.units);
    return std::fprintf(file, "%lu,%s,%s,%.9f,%s,%s,%.9f,%.12f,%.12f\n",
        static_cast<unsigned long>(row.run_id), row.test == Test::dynamic ? "dynamic" : "quasistatic",
        row.direction == Direction::forward ? "forward" : "reverse", row.time_s,
        sample.name.c_str(), output.name(sample.units),
        sample.volts, sample.position / scale, sample.velocity / scale) >= 0;
}

// Independently measured degrees, inch wheel travel. Never infer rotation from width.
class TrackWidthCalibrationInches {
public:
    bool addSpin(double leftStartIn, double leftEndIn, double rightStartIn, double rightEndIn,
                 double headingStartDeg, double headingEndDeg) {
        return si_.addSpin(leftStartIn * metersPerInch, leftEndIn * metersPerInch,
            rightStartIn * metersPerInch, rightEndIn * metersPerInch,
            headingStartDeg * radiansPerDegree, headingEndDeg * radiansPerDegree);
    }
    struct Result { bool valid; std::size_t trials; double trackWidthIn, sampleStddevIn, minWidthIn, maxWidthIn; };
    Result result() const {
        const auto r = si_.result();
        return {r.valid, r.trials, r.track_width_m / metersPerInch, r.sample_stddev_m / metersPerInch,
                r.min_width_m / metersPerInch, r.max_width_m / metersPerInch};
    }
    void clear() { si_.clear(); }
private:
    TrackWidthCalibration si_;
};
} // namespace guanid
