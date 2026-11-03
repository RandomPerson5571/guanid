#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace guanid {

// One uninterrupted in-place spin. Distances are total positive wheel travel
// magnitudes in metres. Rotation is an independently measured, unwrapped angle
// in radians (either sign); it must NOT be inferred from these wheel encoders.
struct TrackWidthTrial {
    double left_distance_m = 0;
    double right_distance_m = 0;
    double rotation_rad = 0;

    double effectiveTrackWidthM() const {
        // Checking if the inputs are finite and positive to avoid invalid calculations.
        if (!std::isfinite(left_distance_m) || left_distance_m <= 0 ||
            !std::isfinite(right_distance_m) || right_distance_m <= 0 ||
            !std::isfinite(rotation_rad) || rotation_rad == 0)
            return std::numeric_limits<double>::quiet_NaN();
        // Divide before adding to avoid overflow in total wheel distance.
        const double angle = std::abs(rotation_rad);
        const double width = left_distance_m/angle + right_distance_m/angle;
        return std::isfinite(width) && width > 0 ? width : std::numeric_limits<double>::quiet_NaN();
    }
};

// Result of repeated trials. Invalid trials leave all accumulated statistics unchanged.
struct TrackWidthResult {
    bool valid = false;
    std::size_t trials = 0;
    double track_width_m = 0;
    // Sample standard deviation across trials; zero for a single trial.
    double sample_stddev_m = 0;
    double min_width_m = 0;
    double max_width_m = 0;
};

// Equal-weight average of repeated trials. No motor commands, tasks, allocation,
// or PROS dependency. Invalid trials leave all accumulated statistics unchanged.
class TrackWidthCalibration {
public:
    /// @brief Add the results of anew trial to the calibration.
    /// @param trial 
    /// @return boolean indicating whether the trial was valid and added successfully.
    bool add(const TrackWidthTrial& trial) {
        const double width = trial.effectiveTrackWidthM();
        if (!std::isfinite(width) || count_ == std::numeric_limits<std::size_t>::max()) return false; // Avoid overflow in count_ and checks if the width is valid.
        const auto count = count_ + 1;
        const double delta = width - mean_;
        const double mean = mean_ + delta/static_cast<double>(count);
        const double m2 = m2_ + delta*(width-mean);
        if (!std::isfinite(mean) || !std::isfinite(m2)) return false; // Check if the new mean and m2 are finite.
        // Update internal state
        mean_ = mean;
        m2_ = m2;
        min_ = count_ ? std::min(min_,width) : width;
        max_ = count_ ? std::max(max_,width) : width;
        count_ = count;
        return true;
    }

    // Convenience for signed, cumulative encoder and gyro snapshots. Both
    // wheel encoders must report forward travel as positive. A spin moves the
    // sides in opposite directions; same-direction or stationary sides fail.
    // Do not reset sensors or reverse the spin between these snapshots.
    bool addSpin(double left_start_m, double left_end_m,
                 double right_start_m, double right_end_m,
                 double heading_start_rad, double heading_end_rad) {
        for (double value : {left_start_m,left_end_m,right_start_m,right_end_m,heading_start_rad,heading_end_rad})
            if (!std::isfinite(value)) return false;
        const double left = left_end_m-left_start_m;
        const double right = right_end_m-right_start_m;
        if (left == 0 || right == 0 || (left > 0) == (right > 0)) return false;
        return add({std::abs(left),std::abs(right),heading_end_rad-heading_start_rad});
    }

    TrackWidthResult result() const {
        return {count_ != 0,count_,mean_,count_ > 1 ? std::sqrt(std::max(0.0,m2_/(count_-1))) : 0,min_,max_};
    }
    void clear() { count_ = 0; mean_ = m2_ = min_ = max_ = 0; }

private:
    std::size_t count_ = 0; // Number of valid trials added.
    double mean_ = 0, m2_ = 0, min_ = 0, max_ = 0;
};

} // namespace guanid
