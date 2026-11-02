#include "guanid/analysis.hpp"
#include <cstdlib>
#include <iostream>

void check(bool v) { if (!v) std::abort(); }
void close(double a,double b,double tolerance=0.005) {
    if (std::abs(a-b)>tolerance) { std::cerr << a << " != " << b << '\n'; std::abort(); }
}
std::vector<guanid::Record> dataset(guanid::Model model, bool noisy = false) {
    using namespace guanid;
    std::vector<Record> rows;
    for (auto test : {Test::quasistatic,Test::dynamic}) for (auto dir : {Direction::forward,Direction::reverse}) {
        const double sign = dir == Direction::forward ? 1 : -1;
        const double wave = test == Test::dynamic ? 0.3 : 0;
        for (int i=0; i<200; ++i) {
            const double t = i*0.02 + (noisy ? 0.002*std::sin(i*1.7) : 0);
            const double v = sign*(0.2+0.4*t+0.07*t*t+wave*std::sin(2*t));
            const double a = sign*(0.4+0.14*t+2*wave*std::cos(2*t));
            const double p = sign*(0.2*t+0.2*t*t+0.07*t*t*t/3+wave*(1-std::cos(2*t))/2);
            MotorSample motor; motor.name="motor";
            motor.voltage(Gains{0.3,1.4,0.45,0.7}.calculate(model,p,v,a));
            const double measured_v = v + (noisy ? 0.001*std::sin(i*2.3) : 0);
            if (model == Model::arm) motor.angularPosition(p).angularVelocity(measured_v);
            else motor.linearPosition(p).linearVelocity(measured_v);
            rows.push_back({0,test,dir,t,motor});
        }
    }
    return rows;
}
int main() {
    using namespace guanid;
    for (auto model : {Model::simple,Model::elevator,Model::arm}) {
        auto rows=dataset(model);
        AnalysisConfig config; config.model=model;
        auto fit=analyze(rows,"motor",config);
        if (!fit.ok) std::cerr << fit.error << '\n';
        check(fit.ok && fit.samples>700 && fit.acceleration_r_squared>0.999);
        close(fit.gains.kS,0.3); close(fit.gains.kV,1.4); close(fit.gains.kA,0.45);
        close(fit.gains.kG,model == Model::simple ? 0 : 0.7);
        check(fit.forward.ok == (model != Model::elevator));
        close(fit.voltage_rmse,0,0.001);
        for (const auto& d : fit.diagnostics) check(std::isfinite(d.residual));
        // A hard stop must never be differentiated across or used as motion.
        for (auto& r : rows) if (r.time_s>3) r.motor.velocity=0;
        fit=analyze(rows,"motor",config); check(fit.ok);
        for (const auto& d : fit.diagnostics) check(d.record.time_s<3);
        config.median_window=3; config.derivative_window=7;
        fit=analyze(dataset(model,true),"motor",config);
        check(fit.ok && fit.acceleration_r_squared>0.98);
        close(fit.gains.kS,0.3,0.02); close(fit.gains.kV,1.4,0.02);
        close(fit.gains.kA,0.45,0.02);
        close(fit.gains.kG,model == Model::simple ? 0 : 0.7,0.02);
    }
    auto rows=dataset(Model::simple);
    auto fit=analyze(rows,"absent"); check(!fit.ok);
    rows[1].time_s=0; check(!analyze(rows,"motor").ok);
    rows=dataset(Model::simple);
    rows.resize(200); check(!analyze(rows,"motor").ok);
    AnalysisConfig config; config.require_four_tests=false;
    check(analyze(rows,"motor",config).ok);
    for (auto& r : rows) r.motor.velocity=1;
    check(!analyze(rows,"motor",config).ok);
    config.median_window=2; check(!analyze(rows,"motor",config).ok);
    rows=dataset(Model::simple);
    config={}; config.model=Model::arm;
    check(!analyze(rows,"motor",config).ok);
    rows[0].motor.volts=std::numeric_limits<double>::quiet_NaN();
    check(!analyze(rows,"motor").ok);

    const Gains gains{0.3,1.4,0.45,0};
    auto velocity=suggestVelocityFeedback(gains,0.1); check(velocity.valid); close(velocity.kP,3.1);
    check(!suggestVelocityFeedback(gains,10).valid);
    check(!suggestVelocityFeedback(gains,0).valid);
    auto position=suggestPositionFeedback(gains,4); check(position.valid); close(position.kP,7.2); close(position.kD,2.2);
    check(!suggestPositionFeedback(gains,-1).valid);

    // Paired channel transformation and missing-pair rejection.
    rows=dataset(Model::simple);
    std::vector<Record> pairs;
    for (auto r : rows) {
        r.motor.name="left"; pairs.push_back(r);
        r.motor.name="right"; r.motor.volts=-r.motor.volts;
        r.motor.velocity=-r.motor.velocity; r.motor.position=-r.motor.position;
        pairs.push_back(r);
    }
    std::vector<Record> derived;
    check(deriveDrivetrain(pairs,"left","right","turn",derived,0.5));
    close(derived[50].motor.velocity,-4*rows[50].motor.velocity);
    close(derived[50].motor.volts,-rows[50].motor.volts);
    check(derived[50].motor.units==Units::angular);
    pairs.pop_back(); check(!deriveDrivetrain(pairs,"left","right","turn",derived,0.5));

    // Timestamp dropouts cannot contribute a derivative across the gap.
    rows=dataset(Model::simple);
    for (auto& r : rows) if (r.time_s>2) r.time_s+=0.5;
    fit=analyze(rows,"motor"); check(fit.ok);
    for (const auto& d : fit.diagnostics) check(d.record.time_s<1.95 || d.record.time_s>2.55);
    // Known asymmetric friction is surfaced to the operator.
    rows=dataset(Model::simple);
    for (auto& r : rows) if (r.direction==Direction::reverse) r.motor.volts-=0.4;
    fit=analyze(rows,"motor"); check(fit.ok);
    bool asymmetry=false;
    for (const auto& warning : fit.warnings) if (warning.find("25 percent")!=std::string::npos) asymmetry=true;
    check(asymmetry);
}
