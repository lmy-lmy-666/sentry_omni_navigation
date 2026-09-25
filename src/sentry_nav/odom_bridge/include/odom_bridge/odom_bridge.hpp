#ifndef ODOM_BRIDGE__ODOM_BRIDGE_HPP_
#define ODOM_BRIDGE__ODOM_BRIDGE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace odom_bridge
{

class OdomBridgeNode : public rclcpp::Node
{
public:
  explicit OdomBridgeNode(const rclcpp::NodeOptions & options);

private:
  void lidarOdometryAndPointCloudCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg);

  // Direct odometry subscription bypasses the ApproximateTime synchronizer.
  // Publishes odom->base_footprint TF and /odometry at high rate regardless
  // of cloud_registered availability.
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  tf2::Transform getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time);

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  void publishOdometry(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  // --- Complementary filter: fuse MCU chassis_yaw with Point-LIO derived yaw ---
  void chassisAttitudeCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void chassisStatusCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  std::string state_estimation_topic_;
  std::string registered_scan_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string robot_base_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom_to_lidar_odom_pub_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> registered_scan_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Direct odometry subscription — decouples TF/odometry publishing from the
  // cloud_registered sync. Rate-limited to avoid flooding at Point-LIO's
  // native ~6500 Hz internal publishing rate.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_direct_sub_;
  std::mutex odom_mutex_;
  tf2::Transform latest_tf_odom_to_lidar_;
  rclcpp::Time latest_odom_stamp_;
  std::chrono::steady_clock::time_point last_tf_publish_time_;
  std::chrono::steady_clock::time_point last_odom_publish_time_;
  static constexpr double MIN_TF_PUBLISH_INTERVAL = 0.01;     // 100 Hz max
  static constexpr double MIN_ODOM_PUBLISH_INTERVAL = 0.02;   // 50 Hz max

  bool base_frame_to_lidar_initialized_;
  tf2::Transform tf_odom_to_lidar_odom_;

  bool has_previous_transform_;
  tf2::Transform previous_transform_;
  std::chrono::steady_clock::time_point previous_time_;

  // EMA smoothing for odom->lidar pose (eliminates Point-LIO micro-jitter in RViz)
  double smoothing_alpha_;
  bool smoothing_initialized_;
  tf2::Transform filtered_transform_;
  double filtered_roll_;
  double filtered_pitch_;
  double filtered_yaw_;

  // EMA smoothing for odometry twist (suppresses finite-difference velocity noise)
  double velocity_smoothing_alpha_;
  bool velocity_smoothing_initialized_;
  double filtered_linear_vel_x_;
  double filtered_linear_vel_y_;
  double filtered_angular_vel_z_;

  // --- Complementary filter: fuse MCU chassis_yaw (gimbal-immune, drifts)
  //     with Point-LIO derived yaw (gimbal-contaminated, no drift) ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr chassis_attitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr chassis_status_sub_;

  std::mutex chassis_state_mutex_;
  double chassis_yaw_mcu_{0.0};       // latest MCU chassis yaw [rad]
  int chassis_mode_{0};               // 0=normal 1=spin_low 2=spin_high 3=estop
  bool chassis_yaw_received_{false};

  // filter state
  bool cf_yaw_initialized_{false};
  double cf_bias_{0.0};               // slow-correcting bias → yaw_clean = yaw_odom + bias
  double cf_yaw_offset_{0.0};         // odom ↔ MCU IMU frame rotation offset
  bool cf_offset_initialized_{false};

  // parameters
  double cf_tau_{15.0};               // time constant [s] — trust odom below this, MCU above
  double cf_offset_gain_{0.0005};     // offset estimation gain (very slow)
  bool cf_enabled_{true};
};

}

#endif
