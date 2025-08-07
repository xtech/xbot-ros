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
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

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
        odom_publisher_ = node.create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

        // Set frame IDs
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        odometry_.header = t.header;
        odometry_.child_frame_id = t.child_frame_id;

        // Pose covariance (6x6 matrix flattened to 36 elements)
        // Order: [x, y, z, roll, pitch, yaw]
        std::array<double, 36> pose_covariance = {{
            0.1,  0.0,  0.0,  0.0,  0.0,  0.0,   // x
            0.0,  0.1,  0.0,  0.0,  0.0,  0.0,   // y
            0.0,  0.0,  0.1,  0.0,  0.0,  0.0,   // z
            0.0,  0.0,  0.0,  0.1,  0.0,  0.0,   // roll
            0.0,  0.0,  0.0,  0.0,  0.1,  0.0,   // pitch
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1    // yaw
        }};

        // Twist covariance (6x6 matrix flattened to 36 elements)
        // Order: [vx, vy, vz, vroll, vpitch, vyaw]
        std::array<double, 36> twist_covariance = {{
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,   // vx
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,   // vy
            0.0,  0.0,  0.05, 0.0,  0.0,  0.0,   // vz
            0.0,  0.0,  0.0,  0.05, 0.0,  0.0,   // vroll
            0.0,  0.0,  0.0,  0.0,  0.05, 0.0,   // vpitch
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05   // vyaw
        }};

        // Copy covariance arrays to message
        std::copy(pose_covariance.begin(), pose_covariance.end(),
                  odometry_.pose.covariance.begin());
        std::copy(twist_covariance.begin(), twist_covariance.end(),
                  odometry_.twist.covariance.begin());

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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    geometry_msgs::msg::TransformStamped t{};
    nav_msgs::msg::Odometry odometry_{};
    // keep the yaw so that we don't have to transform between quaternion and yaw constantly
    double robot_yaw_ = 0.0;
    void CmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg);
    void OnLeftPIDDebugChanged(const float *new_value, uint32_t length) override;

    void OnRightPIDDebugChanged(const float *new_value, uint32_t length) override;

    void OnActualTwistChanged(const double *new_value, uint32_t length) override;
    void OnWheelTicksChanged(const uint32_t *new_value, uint32_t length) override;
};



#endif //DIFF_DRIVE_SERVICE_INTERFACE_HPP
