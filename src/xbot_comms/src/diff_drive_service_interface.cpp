//
// Created by clemens on 7/30/25.
//

#include "diff_drive_service_interface.hpp"
#include <tf2/utils.hpp>

void DiffDriveServiceInterface::OnActualTwistChanged(const double *new_value,
                                                     uint32_t length) {
  if (length != 6) {
    RCLCPP_ERROR(node_.get_logger(), "Invalid length of actual twist: %d",
                 length);
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

  // Get current time
  auto current_time = node_.get_clock()->now();

  // Calculate time difference (dt) since last update
  double dt = 0.0;
  if (t.header.stamp.sec > 0) {
    dt = (current_time - t.header.stamp).seconds();
  }
  t.header.stamp = current_time;
  odometry_.header.stamp = current_time;

  // Extract velocities from the twist message
  double linear_x = new_value[0];  // Linear velocity in x direction (m/s)
  double angular_z = new_value[5]; // Angular velocity around z axis (rad/s)

  // Perform odometry integration if we have a valid time step
  if (dt > 0.0 && dt < 1.0) { // Sanity check: dt should be reasonable
    // For differential drive robots, we typically assume no lateral motion
    // (linear_y = 0)

    // Calculate change in position and orientation
    double delta_x, delta_y, delta_theta;

    if (std::abs(angular_z) < 1e-6) {
      // Pure translation (no rotation)
      delta_x = linear_x * dt;
      delta_y = 0.0;
      delta_theta = 0.0;
    } else {
      // Motion with rotation - use instantaneous center of rotation
      double R = linear_x / angular_z; // Radius of curvature
      delta_theta = angular_z * dt;

      // Calculate position change in robot frame
      delta_x = R * std::sin(delta_theta);
      delta_y = R * (1.0 - std::cos(delta_theta));
    }

    // Transform to global frame
    double cos_theta = std::cos(robot_yaw_);
    double sin_theta = std::sin(robot_yaw_);

    // Update global position
    t.transform.translation.x += cos_theta * delta_x - sin_theta * delta_y;
    t.transform.translation.y += sin_theta * delta_x + cos_theta * delta_y;
    robot_yaw_ += delta_theta;

    // Normalize yaw angle to [-pi, pi]
    while (robot_yaw_ > M_PI)
      robot_yaw_ -= 2.0 * M_PI;
    while (robot_yaw_ < -M_PI)
      robot_yaw_ += 2.0 * M_PI;

    tf2::Quaternion q;
    q.setRPY(0, 0, robot_yaw_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);

    // Also publish odometry message
    odometry_.pose.pose.position.x = t.transform.translation.x;
    odometry_.pose.pose.position.y = t.transform.translation.y;
    odometry_.pose.pose.position.z = 0.0;
    odometry_.pose.pose.orientation = t.transform.rotation;

    // Set twist (velocities in the child frame - base_link)
    odometry_.twist.twist.linear.x = linear_x;
    odometry_.twist.twist.linear.y = 0.0;
    odometry_.twist.twist.linear.z = 0.0;
    odometry_.twist.twist.angular.x = 0.0;
    odometry_.twist.twist.angular.y = 0.0;
    odometry_.twist.twist.angular.z = angular_z;

    odom_publisher_->publish(odometry_);
  } else {
    RCLCPP_WARN(node_.get_logger(),
                "Invalid time step (%f) when updating odometry", dt);
  }
}

void DiffDriveServiceInterface::OnWheelTicksChanged(const uint32_t *new_value,
                                                    uint32_t length) {
  if (length != 2) {
    RCLCPP_ERROR(node_.get_logger(), "Invalid length of wheel ticks: %d",
                 length);
    return;
  }
  std_msgs::msg::UInt32 left_ticks{};
  std_msgs::msg::UInt32 right_ticks{};
  left_ticks.data = new_value[0];
  right_ticks.data = new_value[1];
  left_wheel_ticks_publisher_->publish(left_ticks);
  right_wheel_ticks_publisher_->publish(right_ticks);
}

void DiffDriveServiceInterface::CmdVelReceived(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  double message[6]{};
  message[0] = msg->linear.x;
  message[1] = msg->linear.y;
  message[2] = msg->linear.z;
  message[3] = msg->angular.x;
  message[4] = msg->angular.y;
  message[5] = msg->angular.z;
  SendControlTwist(message, sizeof(message) / sizeof(double));
}

void DiffDriveServiceInterface::OnLeftPIDDebugChanged(const float *new_value,
                                                      uint32_t length) {
  if (length != 3) {
    RCLCPP_ERROR(node_.get_logger(), "Invalid length of left PID debug: %d",
                 length);
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

void DiffDriveServiceInterface::OnRightPIDDebugChanged(const float *new_value,
                                                       uint32_t length) {
  if (length != 3) {
    RCLCPP_ERROR(node_.get_logger(), "Invalid length of right PID debug: %d",
                 length);
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
