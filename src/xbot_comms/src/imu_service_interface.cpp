//
// Created by clemens on 26.07.24.
//

#include "imu_service_interface.h"
#include <service_ids.h>

ImuServiceInterface::ImuServiceInterface(rclcpp::Node &node,
                                         xbot::serviceif::Context ctx)
    : ImuServiceInterfaceBase(xbot::service_ids::IMU, ctx) {
  publisher_ = node.create_publisher<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS());
}

void ImuServiceInterface::OnAxesChanged(const double *new_value,
                                        uint32_t length) {
  if (length < 6) {
    return;
  }
  sensor_msgs::msg::Imu message{};
  message.header.stamp = rclcpp::Clock().now();
  message.header.frame_id = "imu";
  message.linear_acceleration.x = new_value[0];
  message.linear_acceleration.y = new_value[1];
  message.linear_acceleration.z = new_value[2];
  message.angular_velocity.x = new_value[3];
  message.angular_velocity.y = new_value[4];
  message.angular_velocity.z = new_value[5];
  publisher_->publish(message);
}
