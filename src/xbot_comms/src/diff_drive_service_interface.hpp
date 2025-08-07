//
// Created by clemens on 7/30/25.
//

#ifndef DIFF_DRIVE_SERVICE_INTERFACE_HPP
#define DIFF_DRIVE_SERVICE_INTERFACE_HPP

#include <service_ids.h>
#include <geometry_msgs/msg/twist.hpp>
#include "DiffDriveServiceInterfaceBase.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float32.hpp>


class DiffDriveServiceInterface : public DiffDriveServiceInterfaceBase {
public:
    DiffDriveServiceInterface(rclcpp::Node &node, xbot::serviceif::Context ctx)
        : DiffDriveServiceInterfaceBase(xbot::service_ids::DIFF_DRIVE, ctx), node_(node) {
        twist_publisher_ = node.create_publisher<geometry_msgs::msg::Twist>("ll/measured_twist", 10);
        left_wheel_ticks_publisher_ = node.create_publisher<std_msgs::msg::UInt32>("ll/wheel_ticks_left", 10);
        right_wheel_ticks_publisher_ = node.create_publisher<std_msgs::msg::UInt32>("ll/wheel_ticks_right", 10);
        cmd_vel_subscription_ = node.create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&DiffDriveServiceInterface::CmdVelReceived, this, std::placeholders::_1));
        left_pid_debug_input_publisher_ = node.create_publisher<std_msgs::msg::Float32>("ll/diff_drive/left_pid_debug_input", 10);
        left_pid_debug_output_publisher_ = node.create_publisher<std_msgs::msg::Float32>("ll/diff_drive/left_pid_debug_output", 10);
        left_pid_debug_setpoint_publisher_ = node.create_publisher<std_msgs::msg::Float32>("ll/diff_drive/left_pid_debug_setpoint", 10);
        right_pid_debug_input_publisher_ = node.create_publisher<std_msgs::msg::Float32>("ll/diff_drive/right_pid_debug_input", 10);
        right_pid_debug_output_publisher_ = node.create_publisher<std_msgs::msg::Float32>("ll/diff_drive/right_pid_debug_output", 10);
        right_pid_debug_setpoint_publisher_ = node.create_publisher<std_msgs::msg::Float32>("ll/diff_drive/right_pid_debug_setpoint", 10);
    }

private:
    rclcpp::Node &node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr left_wheel_ticks_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr right_wheel_ticks_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_pid_debug_input_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_pid_debug_output_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_pid_debug_setpoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_pid_debug_input_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_pid_debug_output_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_pid_debug_setpoint_publisher_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

    void CmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg);
    void OnLeftPIDDebugChanged(const float *new_value, uint32_t length) override;

    void OnRightPIDDebugChanged(const float *new_value, uint32_t length) override;

    void OnActualTwistChanged(const double *new_value, uint32_t length) override;
    void OnWheelTicksChanged(const uint32_t *new_value, uint32_t length) override;
};



#endif //DIFF_DRIVE_SERVICE_INTERFACE_HPP
