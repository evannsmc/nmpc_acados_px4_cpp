#pragma once

/**
 * C++ wrapper for the acados-generated holybro_euler_err NMPC solver.
 *
 * Model:
 *   State  x (9D): [p(3), v(3), euler(3)]   — ZYX Euler, z-down NED
 *   Control u (4D): [thrust_N, p_rad/s, q_rad/s, r_rad/s]
 *   Params  p (13D): [p_ref(3), v_ref(3), euler_ref(3), u_ref(4)]
 *
 * The solver header is provided by the acados code-generation step;
 * run the Python package first to produce the generated files.
 */

#include <Eigen/Dense>
#include <stdexcept>
#include <string>

// C linkage for acados generated header
extern "C" {
#include "acados_solver_holybro_euler_err.h"
#include "acados_c/ocp_nlp_interface.h"
}

namespace nmpc_acados_px4_cpp {

static constexpr int NMPC_NX = HOLYBRO_EULER_ERR_NX;    ///< 9
static constexpr int NMPC_NU = HOLYBRO_EULER_ERR_NU;    ///< 4
static constexpr int NMPC_NP = HOLYBRO_EULER_ERR_NP;    ///< 13
static constexpr int NMPC_N  = HOLYBRO_EULER_ERR_N;     ///< 50

/// Eigen types (RowMajor so rows map directly to flat C arrays)
using NmpcStateVec  = Eigen::Matrix<double, NMPC_NX, 1>;
using NmpcRefMatrix = Eigen::Matrix<double, NMPC_N, NMPC_NX, Eigen::RowMajor>;
using NmpcCtrlMat   = Eigen::Matrix<double, NMPC_N, NMPC_NU, Eigen::RowMajor>;

struct NmpcResult {
    NmpcCtrlMat simU;   ///< (N×4) optimal control sequence
    int         status; ///< 0 = success (ACADOS_SUCCESS)
};

class NmpcSolver {
public:
    /// @param mass         Quadrotor mass [kg]
    /// @param hover_thrust Nominal hover thrust [N] = mass * g
    NmpcSolver(double mass, double hover_thrust);
    ~NmpcSolver();

    NmpcSolver(const NmpcSolver&)            = delete;
    NmpcSolver& operator=(const NmpcSolver&) = delete;

    /**
     * Solve one SQP_RTI step.
     *
     * @param x0  Current 9D state [p, v, euler], yaw may be unwrapped.
     * @param xd  Reference horizon (NMPC_N × NMPC_NX).
     *            Each row: [x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref,
     *                       roll_ref, pitch_ref, yaw_ref]
     *            roll_ref and pitch_ref are typically zero.
     * @return    Optimal control sequence (N×4) + solver status.
     */
    NmpcResult solve(const NmpcStateVec& x0, const NmpcRefMatrix& xd);

private:
    holybro_euler_err_solver_capsule* capsule_    = nullptr;
    ocp_nlp_config*                   nlp_config_ = nullptr;
    ocp_nlp_dims*                     nlp_dims_   = nullptr;
    ocp_nlp_in*                       nlp_in_     = nullptr;
    ocp_nlp_out*                      nlp_out_    = nullptr;
    ocp_nlp_solver*                   nlp_solver_ = nullptr;

    double mass_;
    double hover_thrust_;
};

}  // namespace nmpc_acados_px4_cpp
