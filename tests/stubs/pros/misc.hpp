#pragma once
#include <functional>
namespace pros {
enum controller_digital_e_t { E_CONTROLLER_DIGITAL_B, E_CONTROLLER_DIGITAL_L1, E_CONTROLLER_DIGITAL_X };
enum controller_analog_e_t { E_CONTROLLER_ANALOG_LEFT_Y, E_CONTROLLER_ANALOG_RIGHT_X };
class Controller {
public:
    std::function<bool(controller_digital_e_t)> digital = [](controller_digital_e_t) { return false; };
    std::function<int(controller_analog_e_t)> analog = [](controller_analog_e_t) { return 0; };
    template <typename... Args> void print(int, int, const char*, Args...) {}
    bool connected = true;
    bool is_connected() const { return connected; }
    bool get_digital(controller_digital_e_t b) const { return digital(b); }
    int get_analog(controller_analog_e_t a) const { return analog(a); }
};
namespace competition {
inline bool disabled = false, autonomous = false;
inline bool is_disabled() { return disabled; }
inline bool is_autonomous() { return autonomous; }
}
}
