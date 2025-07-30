#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "xbot_comms_node.h"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<XbotCommsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
