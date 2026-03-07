#include "nmpc_acados_px4_cpp/nmpc_solver.hpp"

#include <cstring>
#include <stdexcept>
#include <string>

namespace nmpc_acados_px4_cpp {

NmpcSolver::NmpcSolver(double mass, double hover_thrust)
    : mass_(mass), hover_thrust_(hover_thrust)
{
    capsule_ = holybro_euler_err_acados_create_capsule();
    if (!capsule_)
        throw std::runtime_error("[NmpcSolver] holybro_euler_err_acados_create_capsule() returned null");

    int status = holybro_euler_err_acados_create_with_discretization(capsule_, NMPC_N, nullptr);
    if (status)
        throw std::runtime_error("[NmpcSolver] acados_create failed with status " + std::to_string(status));

    nlp_config_ = holybro_euler_err_acados_get_nlp_config(capsule_);
    nlp_dims_   = holybro_euler_err_acados_get_nlp_dims(capsule_);
    nlp_in_     = holybro_euler_err_acados_get_nlp_in(capsule_);
    nlp_out_    = holybro_euler_err_acados_get_nlp_out(capsule_);
    nlp_solver_ = holybro_euler_err_acados_get_nlp_solver(capsule_);
}

NmpcSolver::~NmpcSolver()
{
    if (capsule_) {
        holybro_euler_err_acados_free(capsule_);
        holybro_euler_err_acados_free_capsule(capsule_);
        capsule_ = nullptr;
    }
}

NmpcResult NmpcSolver::solve(const NmpcStateVec& x0, const NmpcRefMatrix& xd)
{
    // ---- initial state constraint ----
    double lbx0[NMPC_NX], ubx0[NMPC_NX];
    for (int i = 0; i < NMPC_NX; ++i) {
        lbx0[i] = x0[i];
        ubx0[i] = x0[i];
    }
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                  0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                  0, "ubx", ubx0);

    // hover control reference [thrust, 0, 0, 0]
    const double u_ref[NMPC_NU] = {hover_thrust_, 0.0, 0.0, 0.0};

    // ---- set stage parameters [p_ref(3), v_ref(3), euler_ref(3), u_ref(4)] ----
    double param[NMPC_NP];
    for (int i = 0; i < NMPC_N; ++i) {
        // p_ref
        param[0] = xd(i, 0);
        param[1] = xd(i, 1);
        param[2] = xd(i, 2);
        // v_ref
        param[3] = xd(i, 3);
        param[4] = xd(i, 4);
        param[5] = xd(i, 5);
        // euler_ref: [roll_ref, pitch_ref, yaw_ref]
        param[6] = xd(i, 6);
        param[7] = xd(i, 7);
        param[8] = xd(i, 8);
        // u_ref
        param[9]  = u_ref[0];
        param[10] = u_ref[1];
        param[11] = u_ref[2];
        param[12] = u_ref[3];
        holybro_euler_err_acados_update_params(capsule_, i, param, NMPC_NP);
    }

    // ---- terminal stage parameters (same as last trajectory step) ----
    {
        const int last = NMPC_N - 1;
        param[0] = xd(last, 0); param[1] = xd(last, 1); param[2] = xd(last, 2);
        param[3] = xd(last, 3); param[4] = xd(last, 4); param[5] = xd(last, 5);
        param[6] = xd(last, 6); param[7] = xd(last, 7); param[8] = xd(last, 8);
        param[9]  = u_ref[0]; param[10] = u_ref[1];
        param[11] = u_ref[2]; param[12] = u_ref[3];
        holybro_euler_err_acados_update_params(capsule_, NMPC_N, param, NMPC_NP);
    }

    // ---- solve ----
    int status = holybro_euler_err_acados_solve(capsule_);

    // ---- extract control horizon ----
    NmpcResult result;
    result.status = status;

    double u_buf[NMPC_NU];
    for (int i = 0; i < NMPC_N; ++i) {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", u_buf);
        for (int j = 0; j < NMPC_NU; ++j)
            result.simU(i, j) = u_buf[j];
    }

    return result;
}

}  // namespace nmpc_acados_px4_cpp
