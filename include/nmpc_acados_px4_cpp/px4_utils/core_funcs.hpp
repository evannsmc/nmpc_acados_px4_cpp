#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

namespace nmpc_acados_px4_cpp::px4_utils {

template<typename NodeT>
void publish_vehicle_command(NodeT& node, uint16_t command,
                              float p1 = 0.0f, float p2 = 0.0f,
                              float p3 = 0.0f, float p4 = 0.0f,
                              double p5 = 0.0, double p6 = 0.0,
                              float p7 = 0.0f) {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.command          = command;
    msg.param1           = p1;
    msg.param2           = p2;
    msg.param3           = p3;
    msg.param4           = p4;
    msg.param5           = p5;
    msg.param6           = p6;
    msg.param7           = p7;
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = node.get_clock()->now().nanoseconds() / 1000;
    node.vehicle_command_publisher->publish(msg);
}

template<typename NodeT>
void arm(NodeT& node) {
    publish_vehicle_command(node,
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(node.get_logger(), "Arm command sent");
}

template<typename NodeT>
void disarm(NodeT& node) {
    publish_vehicle_command(node,
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    RCLCPP_INFO(node.get_logger(), "Disarm command sent");
}

template<typename NodeT>
void engage_offboard_mode(NodeT& node) {
    publish_vehicle_command(node,
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    RCLCPP_INFO(node.get_logger(), "Switching to offboard mode");
}

template<typename NodeT>
void land(NodeT& node) {
    publish_vehicle_command(node,
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(node.get_logger(), "Switching to land mode");
}

template<typename NodeT>
void publish_offboard_heartbeat_position(NodeT& node) {
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.position    = true;
    msg.velocity    = false;
    msg.acceleration= false;
    msg.attitude    = false;
    msg.body_rate   = false;
    msg.timestamp   = node.get_clock()->now().nanoseconds() / 1000;
    node.offboard_control_mode_publisher->publish(msg);
}

template<typename NodeT>
void publish_offboard_heartbeat_bodyrate(NodeT& node) {
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.position    = false;
    msg.velocity    = false;
    msg.acceleration= false;
    msg.attitude    = false;
    msg.body_rate   = true;
    msg.timestamp   = node.get_clock()->now().nanoseconds() / 1000;
    node.offboard_control_mode_publisher->publish(msg);
}

}  // namespace nmpc_acados_px4_cpp::px4_utils
