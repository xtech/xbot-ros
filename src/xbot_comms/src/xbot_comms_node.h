//
// Created by clemens on 7/30/25.
//

#ifndef XBOTCOMMSNODE_H
#define XBOTCOMMSNODE_H

#include <rclcpp/node.hpp>
#include <xbot-service-interface/XbotServiceInterface.hpp>

#include "diff_drive_service_interface.hpp"
#include "imu_service_interface.h"

class XbotCommsNode final : public rclcpp::Node {
public:
    XbotCommsNode();
    void Shutdown();

private:
    xbot::serviceif::Context ctx_;
    std::string bind_ip_;

    std::unique_ptr<ImuServiceInterface> imu_service_interface_;
    std::unique_ptr<DiffDriveServiceInterface> diff_drive_service_interface_;
};



#endif //XBOTCOMMSNODE_H
