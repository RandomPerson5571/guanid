#include "guanid/track_width.hpp"
#include "guanid/analysis.hpp"
#include <cstdlib>
#include <limits>

void check(bool value) { if (!value) std::abort(); }
void close(double actual, double expected) { check(std::abs(actual-expected)<1e-12); }

int main() {
    using namespace guanid;
    constexpr double pi = 3.14159265358979323846;
    constexpr double angle = 10*2*pi;
    TrackWidthCalibration calibration;
    check(!calibration.result().valid && calibration.result().trials == 0);
    // 10 measured turns, with each wheel travelling half the effective width
    // times the total rotation. Asymmetric travel still uses their sum.
    const TrackWidthTrial trial{0.14*angle,0.18*angle,angle};
    close(trial.effectiveTrackWidthM(),0.32);
    check(calibration.add(trial));
    check(calibration.result().valid && calibration.result().trials == 1);
    close(calibration.result().track_width_m,0.32);
    close(calibration.result().sample_stddev_m,0);
    // Equal-weight trial averaging, including a clockwise spin.
    check(calibration.add({0.15*angle,0.15*angle,-angle}));
    check(calibration.add({0.17*angle,0.17*angle,angle}));
    const auto result = calibration.result();
    check(result.trials == 3);
    close(result.track_width_m,0.32);
    close(result.sample_stddev_m,0.02);
    close(result.min_width_m,0.30); close(result.max_width_m,0.34);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const auto& invalid : {
        TrackWidthTrial{0,1,1}, TrackWidthTrial{-1,1,1}, TrackWidthTrial{1,0,1},
        TrackWidthTrial{1,-1,1}, TrackWidthTrial{1,1,0}, TrackWidthTrial{nan,1,1},
        TrackWidthTrial{1,inf,1}, TrackWidthTrial{1,1,nan}, TrackWidthTrial{1,1,inf},
        TrackWidthTrial{1,1,std::numeric_limits<double>::denorm_min()}}) {
        check(!std::isfinite(invalid.effectiveTrackWidthM()));
        check(!calibration.add(invalid));
    }
    check(calibration.result().trials == 3);
    close(calibration.result().track_width_m,result.track_width_m);
    close(calibration.result().sample_stddev_m,result.sample_stddev_m);

    calibration.clear();
    check(!calibration.result().valid);
    // Nonzero encoder/gyro origins; heading is continuous through all 10 turns.
    check(calibration.addSpin(3,3-0.16*angle,7,7+0.16*angle,2,2+angle));
    check(calibration.addSpin(3,3+0.16*angle,7,7-0.16*angle,2,2-angle));
    close(calibration.result().track_width_m,0.32);
    check(!calibration.addSpin(0,1,0,1,0,angle)); // straight travel
    check(!calibration.addSpin(0,0,0,1,0,angle)); // pivot around stopped wheel
    check(!calibration.addSpin(0,-1,0,1,0,0)); // wrapped heading after full turns
    check(!calibration.addSpin(nan,-1,0,1,0,angle));
    check(calibration.result().trials == 2);

    // The calibrated result feeds angular channel derivation directly.
    MotorSample left, right;
    left.name="left"; left.voltage(-2).linearPosition(-0.16*angle).linearVelocity(-0.16);
    right.name="right"; right.voltage(2).linearPosition(0.16*angle).linearVelocity(0.16);
    std::vector<Record> records{{0,Test::dynamic,Direction::forward,1,left},
                                {0,Test::dynamic,Direction::forward,1,right}}, derived;
    check(deriveDrivetrain(records,"left","right","turn",derived,calibration.result().track_width_m));
    close(derived.front().motor.position,angle);
    close(derived.front().motor.velocity,1);
}
