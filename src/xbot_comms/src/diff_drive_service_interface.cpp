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
