#include "control/flight_state_machine.hpp"

namespace dart::control {

const char *ModeName(FlightMode mode) {
    switch (mode) {
        case FlightMode::kFreefall: return "freefall";
        case FlightMode::kGlide: return "glide";
        case FlightMode::kTerminal: return "terminal";
    }
    return "?";
}

} // namespace dart::control
