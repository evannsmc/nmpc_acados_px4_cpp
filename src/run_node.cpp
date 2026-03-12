#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ros2_logger_cpp/log_path.hpp>

#include "quad_platforms_cpp/platform_config.hpp"
#include "quad_trajectories_cpp/types.hpp"
#include "nmpc_acados_px4_cpp/offboard_control_node.hpp"

namespace qp = quad_platforms_cpp;
namespace qt = quad_trajectories_cpp;

// ---------------------------------------------------------------------------
// Argument struct
// ---------------------------------------------------------------------------

struct Args {
    qp::PlatformType      platform;
    qt::TrajectoryType    trajectory;
    std::optional<int>    hover_mode;
    bool   double_speed = false;
    bool   short_variant= false;
    bool   spin         = false;
    bool   feedforward  = false;
    bool   log          = false;
    std::string log_file;
    std::optional<double> flight_period;
};

// ---------------------------------------------------------------------------
// Log filename generator
// ---------------------------------------------------------------------------

static std::string generate_log_filename(const Args& args) {
    std::string name = qp::platform_type_to_string(args.platform);
    name += "_nmpc_acados_px4_";
    name += qt::trajectory_type_to_string(args.trajectory);
    if (args.feedforward)   name += "_ff";
    name += args.double_speed ? "_2x" : "_1x";
    if (args.short_variant) name += "_short";
    if (args.spin)          name += "_spin";
    name += "_cpp";
    return name;
}

// ---------------------------------------------------------------------------
// Usage / argument parsing
// ---------------------------------------------------------------------------

static void print_usage() {
    std::cerr <<
        "\nUsage: ros2 run nmpc_acados_px4_cpp run_node [OPTIONS]\n"
        "\n"
        "NMPC Acados Quadrotor Controller (C++) — Euler state, wrapped yaw error\n"
        "\n"
        "Required:\n"
        "  --platform {sim,hw}         Platform type\n"
        "  --trajectory TRAJ           Trajectory type (see below)\n"
        "\n"
        "Optional:\n"
        "  --hover-mode N              Hover mode index (required when trajectory=hover)\n"
        "  --double-speed              Use 2x trajectory speed\n"
        "  --short                     Use short variant (fig8_vert)\n"
        "  --spin                      Enable spin (circle_horz, helix)\n"
        "  --ff                        Enable differential-flatness feedforward (f8_contraction only)\n"
        "  --flight-period SECONDS     Override default duration (sim: 30s, hw: 60s)\n"
        "  --log                       Enable CSV data logging\n"
        "  --log-file NAME             Custom log filename stem (requires --log)\n"
        "\n"
        "Trajectories: hover, yaw_only, circle_horz, circle_vert,\n"
        "              fig8_horz, fig8_vert, helix, sawtooth, triangle\n"
        "\n"
        "NOTE: The acados generated solver must exist before building.\n"
        "      Run the Python package first:\n"
        "        ros2 run nmpc_acados_px4 run_node --platform sim --trajectory hover --hover-mode 1\n"
        "\n"
        "Examples:\n"
        "  ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory helix --spin\n"
        "  ros2 run nmpc_acados_px4_cpp run_node --platform hw  --trajectory circle_horz --log\n"
        "\n";
}

static Args parse_args(int argc, char* argv[]) {
    Args args;
    bool has_platform    = false;
    bool has_trajectory  = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--platform" && i + 1 < argc) {
            args.platform    = qp::platform_type_from_string(argv[++i]);
            has_platform     = true;
        } else if (arg == "--trajectory" && i + 1 < argc) {
            args.trajectory  = qt::trajectory_type_from_string(argv[++i]);
            has_trajectory   = true;
        } else if (arg == "--hover-mode" && i + 1 < argc) {
            args.hover_mode  = std::stoi(argv[++i]);
        } else if (arg == "--double-speed") {
            args.double_speed= true;
        } else if (arg == "--short") {
            args.short_variant= true;
        } else if (arg == "--spin") {
            args.spin        = true;
        } else if (arg == "--ff") {
            args.feedforward = true;
        } else if (arg == "--log") {
            args.log         = true;
        } else if (arg == "--log-file" && i + 1 < argc) {
            args.log_file    = argv[++i];
        } else if (arg == "--flight-period" && i + 1 < argc) {
            args.flight_period = std::stod(argv[++i]);
        } else {
            print_usage();
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (!has_platform) {
        print_usage();
        throw std::runtime_error("--platform is required");
    }
    if (!has_trajectory) {
        print_usage();
        throw std::runtime_error("--trajectory is required");
    }
    if (args.trajectory == qt::TrajectoryType::HOVER && !args.hover_mode.has_value()) {
        throw std::runtime_error("--hover-mode is required when --trajectory=hover");
    }
    if (args.feedforward && args.trajectory != qt::TrajectoryType::F8_CONTRACTION) {
        throw std::runtime_error("--ff is only valid with --trajectory=f8_contraction");
    }
    if (!args.log_file.empty() && !args.log) {
        throw std::runtime_error("--log-file requires --log");
    }

    return args;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    Args args;
    try {
        args = parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    // Resolve log file path
    std::string log_file;
    if (args.log) {
        std::string fname = args.log_file.empty()
            ? generate_log_filename(args)
            : args.log_file;
        std::filesystem::path exe_path = std::filesystem::canonical(argv[0]);
        auto paths   = ros2_logger_cpp::resolve_log_path(fname, exe_path);
        log_file     = paths.log_file.string();
        ros2_logger_cpp::copy_analysis_helpers(paths.data_analysis);
    }

    // Configuration banner
    const std::string sep(60, '=');
    std::cout << "\n" << sep << "\n";
    std::cout << "NMPC Acados C++ Controller Configuration\n";
    std::cout << sep << "\n";
    std::cout << "Controller:    NMPC Euler-state + wrapped yaw error (C++)\n";
    std::cout << "Platform:      " << qp::platform_type_to_string(args.platform) << "\n";
    std::cout << "Trajectory:    " << qt::trajectory_type_name(args.trajectory) << "\n";
    if (args.hover_mode.has_value())
        std::cout << "Hover mode:    " << *args.hover_mode << "\n";
    std::cout << "Speed:         " << (args.double_speed ? "Double (2x)" : "Regular (1x)") << "\n";
    std::cout << "Short:         " << (args.short_variant ? "Enabled" : "Disabled") << "\n";
    std::cout << "Spin:          " << (args.spin ? "Enabled" : "Disabled") << "\n";
    std::cout << "Feedforward:   " << (args.feedforward ? "Enabled (diff. flatness)" : "Disabled") << "\n";
    double fp = args.flight_period.value_or(
        args.platform == qp::PlatformType::HARDWARE ? 60.0 : 30.0);
    std::cout << "Flight period: " << fp << " s\n";
    std::cout << "Data logging:  "
              << (args.log ? "ON -> " + log_file : "OFF") << "\n";
    std::cout << sep << "\n\n";

    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<nmpc_acados_px4_cpp::OffboardControlNode>(
            args.platform,
            args.trajectory,
            args.hover_mode,
            args.double_speed,
            args.short_variant,
            args.spin,
            args.log,
            log_file,
            args.flight_period,
            args.feedforward);

        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    std::cout << "\nNode shut down." << std::endl;
    return 0;
}
