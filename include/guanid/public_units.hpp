#pragma once

// Robot-facing inches/degrees boundary. The regression and legacy API stay SI.
#include "guanid/analysis.hpp"

namespace guanid {
inline constexpr double metersPerInch = 0.0254;
inline constexpr double radiansPerDegree = 3.14159265358979323846 / 180;
inline double siPerPublicUnit(Units dimension) {
    return dimension == Units::linear ? metersPerInch : radiansPerDegree;
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
    const char* positionUnit() const { return si.units == Units::linear ? "in" : "deg"; }
    Gains gains() const { return gainsInPublicUnits(si.gains, si.units); }
    // Convert only the arm's angle back to radians for cos(). Converted kV/kA
    // already expect public velocity/acceleration; scaling those inputs again
    // would apply the unit conversion twice. Check si.ok before using the result.
    double calculate(double position, double velocity, double acceleration) const {
        const auto g = gains();
        return g.calculate(si.model, si.units == Units::angular ? position * radiansPerDegree : position,
                           velocity, acceleration);
    }
};

// Records/config are explicitly SI, preserving the existing fitter and thresholds.
// Output gains and calculate() use inches or degrees, with volts output.
inline PublicAnalysis analyzePublic(const std::vector<Record>& records, const std::string& motor,
                                    const AnalysisConfig& configSI = {}) {
    return {analyze(records, motor, configSI)};
}

// Same named CSV schema, with explicit in/deg tokens; legacy writeRecord stays SI.
inline bool writePublicRecord(std::FILE* file, const Record& row) {
    if (!file || !row.motor.valid() || !std::isfinite(row.time_s) || row.time_s < 0 ||
        (row.test != Test::quasistatic && row.test != Test::dynamic) ||
        (row.direction != Direction::forward && row.direction != Direction::reverse)) return false;
    const auto& sample = row.motor;
    const double scale = siPerPublicUnit(sample.units);
    return std::fprintf(file, "%lu,%s,%s,%.9f,%s,%s,%.9f,%.12f,%.12f\n",
        static_cast<unsigned long>(row.run_id), row.test == Test::dynamic ? "dynamic" : "quasistatic",
        row.direction == Direction::forward ? "forward" : "reverse", row.time_s,
        sample.name.c_str(), sample.units == Units::linear ? "in" : "deg",
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
