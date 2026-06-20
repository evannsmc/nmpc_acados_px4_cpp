#include "nmpc_acados_px4_cpp/offboard_control_node.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace std::chrono_literals;
namespace qp  = quad_platforms_cpp;
namespace qt  = quad_trajectories_cpp;
namespace pu  = nmpc_acados_px4_cpp::px4_utils;

namespace nmpc_acados_px4_cpp {

// ============================================================================
// Constructor
// ============================================================================

OffboardControlNode::OffboardControlNode(
    qp::PlatformType      platform_type,
    qt::TrajectoryType    trajectory,
    std::optional<int>    hover_mode,
    bool                  double_speed,
    bool                  short_variant,
    bool                  spin,
    bool                  logging_enabled,
    std::string           log_file,
    std::optional<double> flight_period_override,
    bool                  feedforward)
: rclcpp::Node("nmpc_euler_err_offboard_node"),
  sim_(platform_type == qp::PlatformType::SIM),
  platform_type_(platform_type),
  trajectory_type_(trajectory),
  hover_mode_(hover_mode),
  double_speed_(double_speed),
  short_variant_(short_variant),
  spin_(spin),
  logging_enabled_(logging_enabled),
  log_file_(std::move(log_file)),
  feedforward_(feedforward),
  offboard_mode_rc_switch_on_(sim_)   // sim: default ON; hw: wait for RC
{
    // ── Platform ────────────────────────────────────────────────────────────
    platform_ = qp::create_platform(platform_type_);
    RCLCPP_INFO(get_logger(), "Platform: %s  mass=%.3f kg",
        qp::platform_type_to_string(platform_type_).c_str(),
        platform_->mass());

    // ── Heights ──────────────────────────────────────────────────────────
    HOVER_HEIGHT_ = sim_ ? 3.0 : 0.7;
    LAND_HEIGHT_  = sim_ ? 0.6 : 0.45;

    // ── Flight phase timing ──────────────────────────────────────────────
    flight_period_ = flight_period_override.value_or(sim_ ? 30.0 : 60.0);
    land_time_     = flight_period_ + 2.0 * cushion_period_;
    T0_            = std::chrono::steady_clock::now();
    flight_phase_  = get_phase();
    RCLCPP_INFO(get_logger(), "Flight time: %.1f s", land_time_);

    // ── QoS ─────────────────────────────────────────────────────────────
    auto qos = rclcpp::QoS(1)
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::TransientLocal)
        .history(rclcpp::HistoryPolicy::KeepLast);

    // ── Publishers ───────────────────────────────────────────────────────
    offboard_control_mode_publisher = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos);
    trajectory_setpoint_publisher = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos);
    rates_setpoint_publisher = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
        "/fmu/in/vehicle_rates_setpoint", qos);
    vehicle_command_publisher = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos);

    // ── Subscribers ──────────────────────────────────────────────────────
    vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            vehicle_odometry_callback(msg);
        });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            vehicle_status_callback(msg);
        });
    rc_channels_sub_ = create_subscription<px4_msgs::msg::RcChannels>(
        "/fmu/out/rc_channels", qos,
        [this](const px4_msgs::msg::RcChannels::SharedPtr msg) {
            rc_channel_callback(msg);
        });
    // ── NMPC solver ──────────────────────────────────────────────────────
    double hover_thrust = platform_->mass() * 9.8;  // Match Gazebo world
    RCLCPP_INFO(get_logger(), "[NMPC] Creating solver (N=%d, Tf=%.1f s)...", NUM_STEPS, HORIZON);
    nmpc_solver_ = std::make_unique<NmpcSolver>(platform_->mass(), hover_thrust);
    RCLCPP_INFO(get_logger(), "[NMPC] Solver ready.");

    // Warm up: one solve to JIT any lazy-init paths
    time_mpc_solve();

    // ── Logging setup ────────────────────────────────────────────────────
    if (logging_enabled_) {
        RCLCPP_INFO(get_logger(), "Data logging ON -> %s", log_file_.c_str());
        csv_logger_ = std::make_unique<ros2_logger_cpp::CsvLogger>();

        // Headers must be added in the same order as the LogCol enum
        static const char* col_names[] = {
            // Metadata
            "platform", "controller", "trajectory",
            "traj_double", "traj_short", "traj_spin",
            // Timing
            "time", "traj_time", "ref_time", "comp_time",
            // State
            "x", "y", "z", "yaw", "vx", "vy", "vz",
            // Reference
            "x_ref", "y_ref", "z_ref", "yaw_ref",
            "vx_ref", "vy_ref", "vz_ref",
            // Control
            "throttle_input", "p_input", "q_input", "r_input",
            // MPC monitor
            "mpc_status", "buffer_idx",
        };
        for (const auto& name : col_names)
            csv_logger_->add_header(name);

        // Constant metadata (written in every row)
        csv_logger_->set_const_string(COL_PLATFORM,    qp::platform_type_to_string(platform_type_));
        csv_logger_->set_const_string(COL_CONTROLLER,  "NMPC");
        csv_logger_->set_const_string(COL_TRAJECTORY,  qt::trajectory_type_to_string(trajectory_type_));
        csv_logger_->set_const_string(COL_TRAJ_DOUBLE, double_speed_ ? "DblSpd" : "NormSpd");
        csv_logger_->set_const_string(COL_TRAJ_SHORT,  short_variant_ ? "Short" : "NotShort");
        csv_logger_->set_const_string(COL_TRAJ_SPIN,   spin_ ? "Spin" : "NoSpin");
    }

    // ── Timers ───────────────────────────────────────────────────────────
    offboard_timer_      = create_timer(100ms,  [this] { offboard_mode_timer_callback(); });
    publish_control_timer_= create_timer(10ms,  [this] { publish_control_timer_callback(); });
    compute_control_timer_= create_timer(10ms,  [this] { compute_control_timer_callback(); });

    if (logging_enabled_)
        data_log_timer_ = create_timer(100ms, [this] { data_log_timer_callback(); });

    // Reset clock after initialization
    T0_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Node initialized successfully.");
}

OffboardControlNode::~OffboardControlNode()
{
    if (logging_enabled_ && csv_logger_ && !log_file_.empty()) {
        std::cout << "Saving log data to " << log_file_ << "..." << std::endl;
        if (csv_logger_->save(log_file_))
            std::cout << "Log saved (" << csv_logger_->num_rows() << " rows)." << std::endl;
        else
            std::cerr << "Failed to save log." << std::endl;
    }
}

// ============================================================================
// Subscriber callbacks
// ============================================================================

void OffboardControlNode::vehicle_odometry_callback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    x_ = msg->position[0];
    y_ = msg->position[1];
    z_ = msg->position[2];

    vx_ = msg->velocity[0];
    vy_ = msg->velocity[1];
    vz_ = msg->velocity[2];

    p_ = msg->angular_velocity[0];
    q_ = msg->angular_velocity[1];
    r_ = msg->angular_velocity[2];

    // Quaternion → Euler (ZYX: xyz order = roll, pitch, yaw)
    const double qw = msg->q[0], qx = msg->q[1], qy = msg->q[2], qz = msg->q[3];

    // roll (x-axis)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll_ = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis)
    double sinp = 2.0 * (qw * qy - qz * qx);
    pitch_ = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

    // yaw (z-axis)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    raw_yaw_ = std::atan2(siny_cosp, cosy_cosp);

    yaw_ = yaw_tracker_.adjust_yaw(raw_yaw_);

    // 9D NMPC state: [x, y, z, vx, vy, vz, roll, pitch, yaw_unwrapped]
    nmpc_state_ << x_, y_, z_, vx_, vy_, vz_, roll_, pitch_, yaw_;

    mocap_initialized_ = true;
}

void OffboardControlNode::vehicle_status_callback(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    in_offboard_mode_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    armed_            = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    in_land_mode_     = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND);
}

void OffboardControlNode::rc_channel_callback(
    const px4_msgs::msg::RcChannels::SharedPtr msg)
{
    offboard_mode_rc_switch_on_ = (msg->channels[mode_channel_ - 1] >= 0.75f);
}

// ============================================================================
// Flight phase helpers
// ============================================================================

px4_utils::FlightPhase OffboardControlNode::get_phase() const
{
    if (program_time_ < cushion_period_)
        return px4_utils::FlightPhase::HOVER;
    if (program_time_ < cushion_period_ + flight_period_)
        return px4_utils::FlightPhase::CUSTOM;
    if (program_time_ < land_time_)
        return px4_utils::FlightPhase::RETURN;
    return px4_utils::FlightPhase::LAND;
}

double OffboardControlNode::time_before_next_phase(px4_utils::FlightPhase phase) const
{
    switch (phase) {
        case px4_utils::FlightPhase::HOVER:  return cushion_period_ - program_time_;
        case px4_utils::FlightPhase::CUSTOM: return (cushion_period_ + flight_period_) - program_time_;
        case px4_utils::FlightPhase::RETURN: return land_time_ - program_time_;
        default: return 0.0;
    }
}

bool OffboardControlNode::killswitch_and_flight_phase()
{
    if (!offboard_mode_rc_switch_on_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "RC channel %d not set to offboard", mode_channel_);
        offboard_setpoint_counter_ = 0;
        return false;
    }

    auto now   = std::chrono::steady_clock::now();
    program_time_ = std::chrono::duration<double>(now - T0_).count();
    flight_phase_ = get_phase();

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "[%.2f s] Phase: %s  (%.2f s until next)",
        program_time_, pu::flight_phase_name(flight_phase_).c_str(),
        time_before_next_phase(flight_phase_));
    return true;
}

bool OffboardControlNode::get_offboard_health() const
{
    bool healthy = true;
    if (!in_offboard_mode_) {
        RCLCPP_WARN(get_logger(), "Not in OFFBOARD mode.");
        healthy = false;
    }
    if (!armed_) {
        RCLCPP_WARN(get_logger(), "Vehicle not ARMED.");
        healthy = false;
    }
    if (!mocap_initialized_) {
        RCLCPP_WARN(get_logger(), "Odometry not received.");
        healthy = false;
    }
    return healthy;
}

// ============================================================================
// Timer callbacks
// ============================================================================

void OffboardControlNode::offboard_mode_timer_callback()
{
    if (!killswitch_and_flight_phase())
        return;

    if (offboard_setpoint_counter_ == 10) {
        pu::engage_offboard_mode(*this);
        pu::arm(*this);
    }
    if (offboard_setpoint_counter_ < 11)
        ++offboard_setpoint_counter_;

    switch (flight_phase_) {
        case px4_utils::FlightPhase::HOVER:
        case px4_utils::FlightPhase::RETURN:
        case px4_utils::FlightPhase::LAND:
            pu::publish_offboard_heartbeat_position(*this);
            break;
        case px4_utils::FlightPhase::CUSTOM:
            pu::publish_offboard_heartbeat_bodyrate(*this);
            break;
    }
}

void OffboardControlNode::publish_control_timer_callback()
{
    // Disarm if landed
    if (in_land_mode_) {
        double abs_z = std::abs(z_);
        double land_thresh = sim_ ? 0.71 : 0.64;
        if (abs_z < land_thresh) {
            RCLCPP_INFO(get_logger(), "Landed, disarming.");
            pu::disarm(*this);
            rclcpp::shutdown();
            return;
        }
    }

    if (!killswitch_and_flight_phase())
        return;
    if (!get_offboard_health())
        return;

    switch (flight_phase_) {
        case px4_utils::FlightPhase::HOVER:
            publish_position_setpoint(0.0, 0.0, -HOVER_HEIGHT_, 0.0);
            break;

        case px4_utils::FlightPhase::CUSTOM:
            if (buffer_valid_) {
                double thrust_N  = control_buffer_(buffer_index_, 0);
                double roll_rate = control_buffer_(buffer_index_, 1);
                double pitch_rate= control_buffer_(buffer_index_, 2);
                double yaw_rate  = control_buffer_(buffer_index_, 3);

                double throttle = platform_->get_throttle_from_force(thrust_N);

                normalized_input_ = {throttle, roll_rate, pitch_rate, yaw_rate};
                publish_rates_setpoint(throttle, roll_rate, pitch_rate, yaw_rate);

                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 300,
                    "buf[%d] thrust=%.2fN throttle=%.3f p=%.3f q=%.3f r=%.3f",
                    buffer_index_, thrust_N, throttle, roll_rate, pitch_rate, yaw_rate);

                buffer_index_ = std::min(buffer_index_ + 1, NUM_STEPS - 1);
            }
            break;

        case px4_utils::FlightPhase::RETURN:
            publish_position_setpoint(0.0, 0.0, -HOVER_HEIGHT_, 0.0);
            break;

        case px4_utils::FlightPhase::LAND:
            publish_position_setpoint(0.0, 0.0, -LAND_HEIGHT_, 0.0);
            if (std::abs(z_) < 0.64)
                pu::land(*this);
            break;
    }
}

void OffboardControlNode::compute_control_timer_callback()
{
    if (!killswitch_and_flight_phase())
        return;
    if (!get_offboard_health())
        return;
    if (get_phase() != px4_utils::FlightPhase::CUSTOM)
        return;

    if (!trajectory_started_) {
        trajectory_T0_     = std::chrono::steady_clock::now();
        trajectory_time_   = 0.0;
        trajectory_started_= true;
    }

    auto now       = std::chrono::steady_clock::now();
    trajectory_time_ = std::chrono::duration<double>(now - trajectory_T0_).count();
    reference_time_  = trajectory_time_;   // T_LOOKAHEAD = 0

    // Build N×9 reference matrix
    reff_ = build_ref_matrix(trajectory_type_, reference_time_);

    // Differential-flatness feedforward from the shared flatness model.
    if (feedforward_) {
        qt::TrajContext ctx;
        ctx.sim          = sim_;
        ctx.hover_mode   = hover_mode_;
        ctx.spin         = spin_;
        ctx.double_speed = (trajectory_type_ == qt::TrajectoryType::FIG8_CONTRACTION)
            ? false
            : double_speed_;
        ctx.short_variant= short_variant_;

        auto ff = qt::generate_feedforward_trajectory(
            trajectory_type_, ctx, reference_time_, HORIZON, NUM_STEPS);

        // Replace euler_ref cols (6=roll, 7=pitch) with feedforward attitudes.
        // x_ff layout: [px,py,pz,vx,vy,vz, f, phi, th, psi]  (indices 7,8)
        for (int i = 0; i < NUM_STEPS; ++i) {
            reff_(i, 6) = ff.x_ff(i, 7);   // phi (roll)
            reff_(i, 7) = ff.x_ff(i, 8);   // th  (pitch)
            // reff_(i, 8) = yaw from build_ref_matrix — leave as-is
        }

        // Build NMPC feedforward control: u = [F (N), p, q, r]
        // flat_to_x_u gives u_ff = [df, dphi, dth, dpsi] in Euler-rate space.
        // Convert [dphi, dth, dpsi] → body rates [p, q, r] via the inverse
        // of the ZYX Euler-rate kinematic matrix T:
        //   [dphi, dth, dpsi]^T = T(roll, pitch) @ [p, q, r]^T
        const double mass = platform_->mass();
        for (int i = 0; i < NUM_STEPS; ++i) {
            const double f_spec   = ff.x_ff(i, 6);   // specific thrust (m/s²)
            const double phi_ff   = ff.x_ff(i, 7);   // roll
            const double pitch_ff = ff.x_ff(i, 8);   // pitch
            const double dphi_ff  = ff.u_ff(i, 1);
            const double dth_ff   = ff.u_ff(i, 2);
            const double dpsi_ff  = ff.u_ff(i, 3);

            const double sr = std::sin(phi_ff),   cr = std::cos(phi_ff);
            const double sp = std::sin(pitch_ff), cp = std::cos(pitch_ff);
            const double tp = sp / cp;

            Eigen::Matrix3d T;
            T << 1.0,  sr * tp,   cr * tp,
                 0.0,  cr,        -sr,
                 0.0,  sr / cp,   cr / cp;

            Eigen::Vector3d euler_rates(dphi_ff, dth_ff, dpsi_ff);
            Eigen::Vector3d body_rates = T.colPivHouseholderQr().solve(euler_rates);

            u_ff_traj_(i, 0) = mass * f_spec;
            u_ff_traj_(i, 1) = body_rates[0];   // p
            u_ff_traj_(i, 2) = body_rates[1];   // q
            u_ff_traj_(i, 3) = body_rates[2];   // r
        }
        u_ff_valid_ = true;
    } else {
        u_ff_valid_ = false;
    }

    // Solve NMPC
    auto t_start   = std::chrono::steady_clock::now();
    NmpcResult res = nmpc_solver_->solve(nmpc_state_, reff_,
                                         u_ff_valid_ ? &u_ff_traj_ : nullptr);
    auto t_end     = std::chrono::steady_clock::now();
    compute_time_  = std::chrono::duration<double>(t_end - t_start).count();
    last_status_   = res.status;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
        "MPC solve: status=%d  dt=%.3f ms  -> %.1f Hz",
        res.status, compute_time_ * 1e3, 1.0 / compute_time_);

    // Update control buffer
    control_buffer_ = res.simU;
    buffer_index_   = 0;
    buffer_valid_   = true;
}

void OffboardControlNode::data_log_timer_callback()
{
    if (flight_phase_ != px4_utils::FlightPhase::CUSTOM || !csv_logger_)
        return;

    csv_logger_->new_row();

    csv_logger_->set(COL_TIME,      program_time_);
    csv_logger_->set(COL_TRAJ_TIME, trajectory_time_);
    csv_logger_->set(COL_REF_TIME,  reference_time_);
    csv_logger_->set(COL_COMP_TIME, compute_time_);

    csv_logger_->set(COL_X,   x_);
    csv_logger_->set(COL_Y,   y_);
    csv_logger_->set(COL_Z,   z_);
    csv_logger_->set(COL_YAW, yaw_);

    csv_logger_->set(COL_VX, vx_);
    csv_logger_->set(COL_VY, vy_);
    csv_logger_->set(COL_VZ, vz_);

    csv_logger_->set(COL_XREF,   reff_(0, 0));
    csv_logger_->set(COL_YREF,   reff_(0, 1));
    csv_logger_->set(COL_ZREF,   reff_(0, 2));
    csv_logger_->set(COL_VXREF,  reff_(0, 3));
    csv_logger_->set(COL_VYREF,  reff_(0, 4));
    csv_logger_->set(COL_VZREF,  reff_(0, 5));
    csv_logger_->set(COL_YAWREF, reff_(0, 8));

    csv_logger_->set(COL_THROTTLE, normalized_input_[0]);
    csv_logger_->set(COL_P_INPUT,  normalized_input_[1]);
    csv_logger_->set(COL_Q_INPUT,  normalized_input_[2]);
    csv_logger_->set(COL_R_INPUT,  normalized_input_[3]);

    csv_logger_->set(COL_MPC_STATUS, static_cast<double>(last_status_));
    csv_logger_->set(COL_BUFFER_IDX, static_cast<double>(buffer_index_));
}

// ============================================================================
// Setpoint publishers
// ============================================================================

void OffboardControlNode::publish_position_setpoint(
    double x, double y, double z, double yaw)
{
    auto msg       = px4_msgs::msg::TrajectorySetpoint();
    msg.position   = {static_cast<float>(x),
                      static_cast<float>(y),
                      static_cast<float>(z)};
    msg.yaw        = static_cast<float>(yaw);
    msg.timestamp  = get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher->publish(msg);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Position setpoint: [%.2f, %.2f, %.2f] yaw=%.2f", x, y, z, yaw);
}

void OffboardControlNode::publish_rates_setpoint(
    double throttle, double roll, double pitch, double yaw)
{
    auto msg          = px4_msgs::msg::VehicleRatesSetpoint();
    msg.roll          = static_cast<float>(roll);
    msg.pitch         = static_cast<float>(pitch);
    msg.yaw           = static_cast<float>(yaw);
    msg.thrust_body[0]= 0.0f;
    msg.thrust_body[1]= 0.0f;
    msg.thrust_body[2]= -static_cast<float>(throttle);   // PX4: negative z = up
    msg.timestamp     = get_clock()->now().nanoseconds() / 1000;
    rates_setpoint_publisher->publish(msg);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Rates setpoint: thr=%.3f p=%.3f q=%.3f r=%.3f", throttle, roll, pitch, yaw);
}

// ============================================================================
// Reference matrix builder
// ============================================================================

NmpcRefMatrix OffboardControlNode::build_ref_matrix(
    qt::TrajectoryType type, double t_ref) const
{
    NmpcRefMatrix xd = NmpcRefMatrix::Zero();

    qt::TrajContext ctx;
    ctx.sim          = sim_;
    ctx.hover_mode   = hover_mode_;
    ctx.spin         = spin_;
    ctx.double_speed = double_speed_;
    ctx.short_variant= short_variant_;

    auto traj_fn = qt::get_trajectory_fn(type);

    for (int i = 0; i < NUM_STEPS; ++i) {
        double t_i = t_ref + i * DT;

        // Position reference: [x, y, z, yaw]
        Eigen::Vector4d pos = traj_fn(t_i, ctx);
        // Velocity reference via autodiff: [vx, vy, vz, yaw_rate]
        Eigen::Vector4d vel = qt::get_velocity(type, t_i, ctx);

        xd(i, 0) = pos[0];   // x_ref
        xd(i, 1) = pos[1];   // y_ref
        xd(i, 2) = pos[2];   // z_ref
        xd(i, 3) = vel[0];   // vx_ref
        xd(i, 4) = vel[1];   // vy_ref
        xd(i, 5) = vel[2];   // vz_ref
        xd(i, 6) = 0.0;      // roll_ref  (flat flight)
        xd(i, 7) = 0.0;      // pitch_ref (flat flight)
        xd(i, 8) = pos[3];   // yaw_ref
    }

    return xd;
}

// ============================================================================
// Initialization timing warm-up
// ============================================================================

void OffboardControlNode::time_mpc_solve()
{
    // Build a dummy reference
    NmpcStateVec  x0 = NmpcStateVec::Zero();
    x0 << 0.01, 0.02, -HOVER_HEIGHT_, 0.01, -0.03, 0.05, 0.01, 0.02, 0.03;

    NmpcRefMatrix xd = NmpcRefMatrix::Zero();
    Eigen::Vector<double, NMPC_NX> ref0;
    ref0 << 3.0, 3.0, -5.0, -0.01, 0.03, 0.05, 0.0, 0.0, -0.01;
    for (int i = 0; i < NUM_STEPS; ++i)
        xd.row(i) = ref0.transpose();

    // Warm-up solves
    for (int k = 0; k < 2; ++k) {
        auto t0  = std::chrono::steady_clock::now();
        auto res = nmpc_solver_->solve(x0, xd);
        auto dt  = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        RCLCPP_INFO(get_logger(),
            "[NMPC warm-up %d] status=%d  u0=[%.2f, %.3f, %.3f, %.3f]  dt=%.3f ms (%.1f Hz)",
            k, res.status,
            res.simU(0,0), res.simU(0,1), res.simU(0,2), res.simU(0,3),
            dt * 1e3, 1.0 / dt);
    }
}

}  // namespace nmpc_acados_px4_cpp
