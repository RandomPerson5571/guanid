#pragma once
#include <cstdint>
namespace pros {
inline std::uint32_t test_clock = 0;
inline std::uint32_t millis() { return test_clock; }
inline void delay(std::uint32_t ms) { test_clock += ms; }
} // namespace pros
