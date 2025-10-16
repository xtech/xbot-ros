//
// Created by clemens on 26.07.24.
//

#ifndef IMUSERVICEINTERFACE_H
#define IMUSERVICEINTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include <ImuServiceInterfaceBase.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuServiceInterface : public ImuServiceInterfaceBase {
public:
  ImuServiceInterface(rclcpp::Node &node, xbot::serviceif::Context ctx);

protected:
  void OnAxesChanged(const double *new_value, uint32_t length) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

#endif // IMUSERVICEINTERFACE_H
