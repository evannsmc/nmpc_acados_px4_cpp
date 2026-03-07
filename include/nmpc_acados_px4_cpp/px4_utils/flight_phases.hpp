#pragma once

#include <string>

namespace nmpc_acados_px4_cpp::px4_utils {

enum class FlightPhase {
    HOVER,
    CUSTOM,
    RETURN,
    LAND,
};

inline std::string flight_phase_name(FlightPhase phase) {
    switch (phase) {
        case FlightPhase::HOVER:  return "HOVER";
        case FlightPhase::CUSTOM: return "CUSTOM";
        case FlightPhase::RETURN: return "RETURN";
        case FlightPhase::LAND:   return "LAND";
    }
    return "UNKNOWN";
}

}  // namespace nmpc_acados_px4_cpp::px4_utils
