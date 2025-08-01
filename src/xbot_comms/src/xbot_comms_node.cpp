//
// Created by clemens on 7/30/25.
//

#include "xbot_comms_node.h"

XbotCommsNode::XbotCommsNode() : Node("xbot_comms") {
    declare_parameter("bind_ip", "0.0.0.0");

    bind_ip_ = get_parameter("bind_ip").as_string();

    RCLCPP_INFO(this->get_logger(), "Bind IP: %s", bind_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "Starting xBot Framework");
    ctx_ = xbot::serviceif::Start(false, bind_ip_);

    imu_service_interface_ = std::make_unique<ImuServiceInterface>(*this, ctx_);
    diff_drive_service_interface_ = std::make_unique<DiffDriveServiceInterface>(*this, ctx_);
    imu_service_interface_->Start();
    diff_drive_service_interface_->Start();
}

void XbotCommsNode::Shutdown() {
    xbot::serviceif::Stop();
}
