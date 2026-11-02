#pragma once
#include "pros/error.h"
#include <cstdint>
#include <vector>

namespace pros {
enum class MotorUnits { degrees, rotations };
// Minimal test double for the PROS calls used by the adapter. Real-header
// compatibility is checked separately by the PROS consumer build.
class AbstractMotor {
public:
    struct Reading {
        double degrees = 0, rpm = 0;
        std::int32_t millivolts = 0;
        MotorUnits units = MotorUnits::degrees;
    };
    std::vector<Reading> readings{Reading{}};
    std::vector<std::int32_t> commands;
    bool fail_drive = false, fail_stop = false;
    std::int8_t size() const { return static_cast<std::int8_t>(readings.size()); }
    MotorUnits get_encoder_units(std::uint8_t i) const { return readings.at(i).units; }
    double get_position(std::uint8_t i) const { return readings.at(i).degrees; }
    double get_actual_velocity(std::uint8_t i) const { return readings.at(i).rpm; }
    std::int32_t get_voltage(std::uint8_t i) const { return readings.at(i).millivolts; }
    std::int32_t move_voltage(std::int32_t mv) {
        commands.push_back(mv);
        if ((mv == 0 && fail_stop) || (mv != 0 && fail_drive)) return PROS_ERR;
        for (auto& reading : readings) reading.millivolts = mv;
        return 1;
    }
};
class Motor : public AbstractMotor {};
} // namespace pros
