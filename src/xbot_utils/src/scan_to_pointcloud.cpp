#include "xbot_utils/scan_to_pointcloud.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_ros/transforms.hpp>

namespace xbot_utils {
    ScanToPointcloud::ScanToPointcloud(const rclcpp::NodeOptions &options)
        : Node("scan_to_pointcloud", options) {
        // Declare and get parameters
        this->declare_parameter("target_frame", "odom");
        this->declare_parameter("max_cloud_size", 10000);

        target_frame_ = this->get_parameter("target_frame").as_string();
        max_cloud_size_ = static_cast<size_t>(this->get_parameter("max_cloud_size").as_int());

        RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Max cloud size: %zu", max_cloud_size_);

        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize point cloud
        cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        // Create publisher and subscriber
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ScanToPointcloud::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Scan to pointcloud node initialized");
    }

    void ScanToPointcloud::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

        if (transformScanToPointCloud(scan, cloud_out)) {
            std::lock_guard<std::mutex> lock(cloud_mutex_);

            // Add new points to the accumulated cloud
            *cloud_ += *cloud_out;

            // Limit cloud size if necessary
            if (cloud_->size() > max_cloud_size_) {
                // Remove oldest points to maintain max size
                cloud_->erase(cloud_->begin(), cloud_->begin() + (cloud_->size() - max_cloud_size_));
            }

            publishPointCloud();
        }
    }

    bool ScanToPointcloud::transformScanToPointCloud(
        const sensor_msgs::msg::LaserScan::SharedPtr &scan,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) {
        // Create a point cloud to hold the laser scan in the laser frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());

        // Convert laser scan to point cloud in laser frame
        // For each point in the laser scan
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];

            // Skip invalid measurements
            if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
                continue;
            }

            // Calculate the angle for this measurement
            float angle = scan->angle_min + i * scan->angle_increment;

            // Convert polar coordinates to Cartesian
            pcl::PointXYZ point;
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            point.z = 0.0; // 2D laser scan, so z is 0

            cloud_in->push_back(point);
        }

        // Set the frame ID for the point cloud
        cloud_in->header.frame_id = scan->header.frame_id;
        pcl_conversions::toPCL(scan->header.stamp, cloud_in->header.stamp);

        geometry_msgs::msg::TransformStamped transform_stamped;
        bool transform_found = false;
        for (int i = 0; i < 10 && !transform_found; ++i) {
            try {
                // Look up transform from laser frame to target frame
                float dt = (this->now() - scan->header.stamp).seconds();
                RCLCPP_INFO_STREAM(this->get_logger(), "Time Diff: " << dt << "sec" );
                transform_stamped =
                        tf_buffer_->lookupTransform(target_frame_, scan->header.frame_id, scan->header.stamp - rclcpp::Duration::from_seconds(scan->scan_time) );
                transform_found = true;
            } catch (tf2::TransformException &ex) {
                // RCLCPP_WARN(this->get_logger(), "Could not transform laser scan: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (transform_found) {
            // Transform point cloud to target frame
            pcl_ros::transformPointCloud(*cloud_in, *cloud_out, transform_stamped);

            // Update the frame ID for the transformed point cloud
            cloud_out->header.frame_id = target_frame_;

            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not transform laser scan");
            return false;
        }
    }

    void ScanToPointcloud::publishPointCloud() {
        // Convert PCL cloud to ROS message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_, cloud_msg);

        // Set header
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = target_frame_;

        // Publish
        cloud_pub_->publish(cloud_msg);
    }
} // namespace xbot_utils
