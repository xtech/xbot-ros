//
// Created by clemens on 7/30/25.
//

#ifndef DIFF_DRIVE_SERVICE_INTERFACE_HPP
#define DIFF_DRIVE_SERVICE_INTERFACE_HPP

#include <service_ids.h>
#include <geometry_msgs/msg/twist.hpp>
#include "DiffDriveServiceInterfaceBase.hpp"
#include <rclcpp/rclcpp.hpp>


class DiffDriveServiceInterface : public DiffDriveServiceInterfaceBase {
protected:
    void OnActualTwistChanged(const double *new_value, uint32_t length) override;

public:
    DiffDriveServiceInterface(rclcpp::Node &node, xbot::serviceif::Context ctx)
        : DiffDriveServiceInterfaceBase(xbot::service_ids::DIFF_DRIVE, ctx), node_(node) {
        twist_publisher_ = node.create_publisher<geometry_msgs::msg::Twist>("ll/measured_twist", 10);
        cmd_vel_subscription_ = node.create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&DiffDriveServiceInterface::CmdVelReceived, this, std::placeholders::_1));
    }

private:
    rclcpp::Node &node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

    void CmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg);
};



#endif //DIFF_DRIVE_SERVICE_INTERFACE_HPP
