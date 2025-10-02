# ScanToPointcloud Node

## Overview

`ScanToPointcloud` is a ROS 2 node that converts incoming 2D laser scans (`sensor_msgs/msg/LaserScan`) into an accumulated 3D point cloud (`sensor_msgs/msg/PointCloud2`). It supports motion compensation (deskewing) using odometry data to improve the accuracy of the resulting point cloud, especially when the robot is moving during a scan.

## Features
- Subscribes to `/scan` (LaserScan) and `/odom` (Odometry) topics
- Buffers odometry messages for accurate deskewing
- Deskews laser scan points using the robot's linear and angular velocity
- Transforms deskewed points into a target frame (default: `odom`)
- Accumulates points into a rolling point cloud of configurable size
- Publishes the accumulated point cloud as a `sensor_msgs/msg/PointCloud2`

## Parameters
- `target_frame` (string, default: `odom`):
  The frame to which all points are transformed before publishing.
- `max_cloud_size` (int, default: 10000):
  The maximum number of points in the accumulated point cloud. Older points are dropped when this limit is exceeded.
- `odom_buffer_duration` (double, default: 0.5):
  The duration (in seconds) for which odometry messages are buffered for deskewing.

## Topics
- **Subscribed:**
  - `/scan` (`sensor_msgs/msg/LaserScan`): Input laser scans
  - `/odom` (`nav_msgs/msg/Odometry`): Odometry for deskewing
- **Published:**
  - `pointcloud` (`sensor_msgs/msg/PointCloud2`): Accumulated, deskewed, and transformed point cloud

## How Deskewing Works
- For each scan, the node finds the closest odometry message by timestamp.
- It estimates the robot's motion (translation and rotation) during the scan using the odometry's twist (velocity).
- Each laser point is rotated and translated backwards in time to compensate for the robot's movement, as if the robot had not moved during the scan.
- The deskewed points are then transformed into the target frame and added to the accumulated point cloud.
- If no odometry is available, deskewing is skipped and points are used as-is.

## Usage Example

```
ros2 run xbot_utils scan_to_pointcloud_node
```

Or include in your launch file:

```xml
<node pkg="xbot_utils" exec="scan_to_pointcloud_node" name="scan_to_pointcloud" output="screen" />
```

## Notes
- The node is robust to missing odometry: if no odometry is available, it will still publish a point cloud, but without deskewing.
- Make sure your TF tree is set up so that the transform from the laser frame to the target frame is available.

## Dependencies
- ROS 2 (rclcpp, sensor_msgs, nav_msgs, tf2_ros)
- PCL (Point Cloud Library)
- pcl_ros
