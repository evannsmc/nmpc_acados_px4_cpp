#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <px4_msgs/msg/battery_status.hpp>

#include <Eigen/Dense>
#include <memory>
#include <array>
#include <chrono>
#include <string>
#include <optional>

#include "quad_platforms_cpp/platform_config.hpp"
#include "quad_trajectories_cpp/types.hpp"
#include "quad_trajectories_cpp/utils.hpp"
#include "quad_trajectories_cpp/registry.hpp"

#include "nmpc_acados_px4_cpp/nmpc_solver.hpp"
#include "nmpc_acados_px4_cpp/px4_utils/flight_phases.hpp"
#include "nmpc_acados_px4_cpp/px4_utils/core_funcs.hpp"
#include "nmpc_acados_px4_cpp/transformations/adjust_yaw.hpp"

#include <ros2_logger_cpp/csv_logger.hpp>

namespace nmpc_acados_px4_cpp {

class OffboardControlNode : public rclcpp::Node {
public:
    OffboardControlNode(
        quad_platforms_cpp::PlatformType         platform_type,
        quad_trajectories_cpp::TrajectoryType    trajectory,
        std::optional<int>                       hover_mode,
        bool                                     double_speed,
        bool                                     short_variant,
        bool                                     spin,
        bool                                     logging_enabled,
        std::string                              log_file,
        std::optional<double>                    flight_period_override);

    ~OffboardControlNode();

    // Public publishers (accessed by core_funcs templates)
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr  offboard_control_mode_publisher;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   trajectory_setpoint_publisher;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_setpoint_publisher;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr       vehicle_command_publisher;

private:
    // ── Subscriber callbacks ─────────────────────────────────────────────
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void rc_channel_callback(const px4_msgs::msg::RcChannels::SharedPtr msg);
    void battery_status_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);

    // ── Timer callbacks ──────────────────────────────────────────────────
    void offboard_mode_timer_callback();
    void publish_control_timer_callback();
    void compute_control_timer_callback();
    void data_log_timer_callback();

    // ── Flight phase helpers ─────────────────────────────────────────────
    px4_utils::FlightPhase get_phase() const;
    double time_before_next_phase(px4_utils::FlightPhase phase) const;
    bool killswitch_and_flight_phase();
    bool get_offboard_health() const;

    // ── Control ──────────────────────────────────────────────────────────
    void controller();
    void publish_position_setpoint(double x, double y, double z, double yaw);
    void publish_rates_setpoint(double throttle, double roll, double pitch, double yaw);

    // ── Trajectory helpers ───────────────────────────────────────────────
    NmpcRefMatrix build_ref_matrix(quad_trajectories_cpp::TrajectoryType type,
                                   double t_ref) const;

    // ── Initialization timing test ───────────────────────────────────────
    void time_mpc_solve();

    // ── Config ───────────────────────────────────────────────────────────
    bool sim_;
    quad_platforms_cpp::PlatformType      platform_type_;
    quad_trajectories_cpp::TrajectoryType trajectory_type_;
    std::optional<int>                    hover_mode_;
    bool double_speed_;
    bool short_variant_;
    bool spin_;
    bool logging_enabled_;
    std::string log_file_;

    std::unique_ptr<quad_platforms_cpp::PlatformConfig> platform_;

    // ── Subscribers ──────────────────────────────────────────────────────
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr   vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr      rc_channels_sub_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr   battery_status_sub_;

    // ── Timers ───────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr offboard_timer_;
    rclcpp::TimerBase::SharedPtr publish_control_timer_;
    rclcpp::TimerBase::SharedPtr compute_control_timer_;
    rclcpp::TimerBase::SharedPtr data_log_timer_;

    // ── Vehicle state (NED frame) ────────────────────────────────────────
    transformations::YawTracker yaw_tracker_;
    bool mocap_initialized_ = false;

    double x_  = 0.0, y_  = 0.0, z_  = 0.0;
    double vx_ = 0.0, vy_ = 0.0, vz_ = 0.0;
    double p_  = 0.0, q_  = 0.0, r_  = 0.0;   ///< body angular rates
    double roll_ = 0.0, pitch_ = 0.0;
    double yaw_    = 0.0;   ///< Unwrapped yaw (via YawTracker)
    double raw_yaw_= 0.0;   ///< Raw yaw ∈ (−π, π]

    bool in_offboard_mode_        = false;
    bool armed_                   = false;
    bool in_land_mode_            = false;
    bool offboard_mode_rc_switch_on_;
    int  mode_channel_            = 5;

    double current_voltage_ = 16.8;   ///< 4S LiPo nominal [V]

    // ── Flight phase timing ──────────────────────────────────────────────
    std::chrono::steady_clock::time_point T0_;
    double program_time_   = 0.0;
    double cushion_period_ = 10.0;
    double flight_period_;
    double land_time_;
    px4_utils::FlightPhase flight_phase_;
    int offboard_setpoint_counter_ = 0;

    // ── Heights ──────────────────────────────────────────────────────────
    double HOVER_HEIGHT_;
    double LAND_HEIGHT_;

    // ── NMPC solver ──────────────────────────────────────────────────────
    static constexpr double HORIZON   = 2.0;
    static constexpr int    NUM_STEPS = NMPC_N;       ///< 50
    static constexpr double DT        = HORIZON / NUM_STEPS; ///< 0.04 s

    std::unique_ptr<NmpcSolver> nmpc_solver_;

    NmpcStateVec  nmpc_state_  = NmpcStateVec::Zero();
    NmpcRefMatrix reff_        = NmpcRefMatrix::Zero();

    // ── Control buffer (MPC horizon playback) ────────────────────────────
    NmpcCtrlMat control_buffer_ = NmpcCtrlMat::Zero();
    int  buffer_index_ = 0;
    bool buffer_valid_ = false;

    std::array<double, 4> normalized_input_ = {0.0, 0.0, 0.0, 0.0};
    double compute_time_ = 0.0;
    int    last_status_  = 0;

    // ── Trajectory timing ────────────────────────────────────────────────
    bool   trajectory_started_ = false;
    std::chrono::steady_clock::time_point trajectory_T0_;
    double trajectory_time_ = 0.0;
    double reference_time_  = 0.0;

    // ── Logging ──────────────────────────────────────────────────────────
    std::unique_ptr<ros2_logger_cpp::CsvLogger> csv_logger_;

    enum LogCol : size_t {
        // Constant metadata
        COL_PLATFORM = 0, COL_CONTROLLER, COL_TRAJECTORY,
        COL_TRAJ_DOUBLE, COL_TRAJ_SHORT, COL_TRAJ_SPIN,
        // Timing
        COL_TIME, COL_TRAJ_TIME, COL_REF_TIME, COL_COMP_TIME,
        // State
        COL_X, COL_Y, COL_Z, COL_YAW,
        COL_VX, COL_VY, COL_VZ,
        // Reference (first horizon step)
        COL_XREF, COL_YREF, COL_ZREF, COL_YAWREF,
        COL_VXREF, COL_VYREF, COL_VZREF,
        // Control inputs (throttle 0-1, rates rad/s)
        COL_THROTTLE, COL_P_INPUT, COL_Q_INPUT, COL_R_INPUT,
        // MPC monitor
        COL_MPC_STATUS, COL_BUFFER_IDX,
        COL_COUNT
    };
};

}  // namespace nmpc_acados_px4_cpp
