#include "guanid/guanid.hpp"
#include <cstdlib>
#include <limits>
#include <string>

void check(bool condition) {
    if (!condition) std::abort();
}

int main() {
    using namespace guanid;
    Config config;
    check(config.valid());
    check(commandVolts(config, Test::quasistatic, Direction::forward, 1000) == 0.5);
    check(commandVolts(config, Test::quasistatic, Direction::reverse, 1000) == -0.5);
    check(commandVolts(config, Test::dynamic, Direction::forward, 0) == 3);
    check(commandVolts(config, Test::dynamic, Direction::reverse, 0) == -3);
    check(commandVolts(config, Test::dynamic, Direction::forward, 3000) == 0);
    config.max_volts = 1;
    check(commandVolts(config, Test::dynamic, Direction::reverse, 0) == -1);
    check(commandVolts(config, Test::quasistatic, Direction::forward, 2500) == 1);
    config.max_volts = 13;
    check(!config.valid());
    config = {};
    config.step_volts = std::numeric_limits<double>::quiet_NaN();
    check(!config.valid());
    config = {};
    config.interval_ms = 0;
    check(!config.valid());
    config = {};
    config.duration_ms = 0;
    check(!config.valid());

    const Feedforward ff{0.2, 2, 0.5};
    check(ff.calculate(1, 2) == 3.2);
    check(ff.calculate(-1, -2) == -3.2);
    check(ff.calculate(0, 2) == 1);

    auto* file = std::tmpfile();
    check(file != nullptr);
    check(writeCsvHeader(file));
    Sample sample{Test::dynamic, Direction::reverse, 1250, -3,
                  {-2.9, -1, -0.5}, {-2.8, -1.1, -0.6}};
    check(writeCsvSample(file, sample));
    std::rewind(file);
    char buffer[512];
    check(std::fgets(buffer, sizeof(buffer), file) != nullptr);
    check(std::string(buffer).find("left_applied_volts") != std::string::npos);
    check(std::fgets(buffer, sizeof(buffer), file) != nullptr);
    check(std::string(buffer) == "dynamic,reverse,1.250000,-3.000000,-2.900000,-1.000000000,-0.500000000,-2.800000,-1.100000000,-0.600000000,0\n");
    std::fclose(file);
    check(!writeCsvHeader(nullptr));
    check(!writeCsvSample(nullptr, sample));
}
