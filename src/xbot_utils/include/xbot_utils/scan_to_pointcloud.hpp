#ifndef XBOT_UTILS_SCAN_TO_POINTCLOUD_HPP
#define XBOT_UTILS_SCAN_TO_POINTCLOUD_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <deque>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>

namespace xbot_utils
{

class ScanToPointcloud : public rclcpp::Node
{
public:
  explicit ScanToPointcloud(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ScanToPointcloud() = default;

private:
  // ROS parameters
  std::string target_frame_;
  size_t max_cloud_size_;
  rclcpp::Duration odom_buffer_duration_ = rclcpp::Duration::from_seconds(10.0);

  // Subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // TF2 objects
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Point cloud storage
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  std::mutex cloud_mutex_;

  // Odometry buffer
  std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_buffer_;
  std::mutex odom_mutex_;

  // Callback for laser scan messages
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  // Transform laser scan to point cloud
  bool transformScanToPointCloud(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_out);

  // Publish the accumulated point cloud
  void publishPointCloud();

  // Odometry callback
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  // Find closest odometry message to a given time
  nav_msgs::msg::Odometry::SharedPtr getClosestOdom(const rclcpp::Time& stamp);

  // Deskew points using twist
  void deskewPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                        const nav_msgs::msg::Odometry::SharedPtr& odom,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_out);
};

}  // namespace xbot_utils

#endif  // XBOT_UTILS_SCAN_TO_POINTCLOUD_HPP