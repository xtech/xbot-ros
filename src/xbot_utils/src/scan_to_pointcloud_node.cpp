#include "xbot_utils/scan_to_pointcloud.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<xbot_utils::ScanToPointcloud>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
