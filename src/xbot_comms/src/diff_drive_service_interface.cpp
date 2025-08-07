//
// Created by clemens on 7/30/25.
//

#include "diff_drive_service_interface.hpp"

void DiffDriveServiceInterface::OnActualTwistChanged(const double *new_value, uint32_t length) {
    if (length != 6) {
        RCLCPP_ERROR(node_.get_logger(), "Invalid length of actual twist: %d", length);
        return;
    }
    geometry_msgs::msg::Twist message{};
    message.linear.x = new_value[0];
    message.linear.y = new_value[1];
    message.linear.z = new_value[2];
    message.angular.x = new_value[3];
    message.angular.y = new_value[4];
    message.angular.z = new_value[5];
    twist_publisher_->publish(message);
}

void DiffDriveServiceInterface::OnWheelTicksChanged(const uint32_t *new_value, uint32_t length) {
    if (length != 2) {
        RCLCPP_ERROR(node_.get_logger(), "Invalid length of wheel ticks: %d", length);
        return;
    }
    std_msgs::msg::UInt32 left_ticks{};
    std_msgs::msg::UInt32 right_ticks{};
    left_ticks.data = new_value[0];
    right_ticks.data = new_value[1];
    left_wheel_ticks_publisher_->publish(left_ticks);
    right_wheel_ticks_publisher_->publish(right_ticks);
}

void DiffDriveServiceInterface::CmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double message[6]{};
    message[0] = msg->linear.x;
    message[1] = msg->linear.y;
    message[2] = msg->linear.z;
    message[3] = msg->angular.x;
    message[4] = msg->angular.y;
    message[5] = msg->angular.z;
    SendControlTwist(message, sizeof(message)/sizeof(double));
}

void DiffDriveServiceInterface::OnLeftPIDDebugChanged(const float *new_value, uint32_t length) {
    if (length != 3) {
        RCLCPP_ERROR(node_.get_logger(), "Invalid length of left PID debug: %d", length);
        return;
    }
    std_msgs::msg::Float32 message{};
    message.data = new_value[0];
    left_pid_debug_input_publisher_->publish(message);
    message.data = new_value[1];
    left_pid_debug_output_publisher_->publish(message);
    message.data = new_value[2];
    left_pid_debug_setpoint_publisher_->publish(message);
}

void DiffDriveServiceInterface::OnRightPIDDebugChanged(const float *new_value, uint32_t length) {
    if (length != 3) {
        RCLCPP_ERROR(node_.get_logger(), "Invalid length of right PID debug: %d", length);
        return;
    }
    std_msgs::msg::Float32 message{};
    message.data = new_value[0];
    right_pid_debug_input_publisher_->publish(message);
    message.data = new_value[1];
    right_pid_debug_output_publisher_->publish(message);
    message.data = new_value[2];
    right_pid_debug_setpoint_publisher_->publish(message);
}
