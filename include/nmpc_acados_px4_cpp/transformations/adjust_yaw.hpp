#pragma once

#include <cmath>

namespace nmpc_acados_px4_cpp::transformations {

/// Tracks unwrapped yaw by counting full rotations across the ±π discontinuity.
struct YawTracker {
    bool   initialized    = false;
    double prev_mocap_psi = 0.0;
    int    full_rotations = 0;

    double adjust_yaw(double yaw) {
        double mocap_psi = yaw;

        if (!initialized) {
            initialized    = true;
            prev_mocap_psi = mocap_psi;
            return mocap_psi;
        }

        if (prev_mocap_psi >  M_PI * 0.9 && mocap_psi < -M_PI * 0.9)
            full_rotations += 1;
        else if (prev_mocap_psi < -M_PI * 0.9 && mocap_psi >  M_PI * 0.9)
            full_rotations -= 1;

        double psi     = mocap_psi + 2.0 * M_PI * full_rotations;
        prev_mocap_psi = mocap_psi;
        return psi;
    }
};

}  // namespace nmpc_acados_px4_cpp::transformations
