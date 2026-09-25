#include "odom_bridge/odom_bridge.hpp"

#include <cmath>

#include "pcl_ros/transforms.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
/// Normalize angle to [-pi, pi)
inline double normAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

namespace odom_bridge
{

OdomBridgeNode::OdomBridgeNode(const rclcpp::NodeOptions & options)
: Node("odom_bridge", options),
  base_frame_to_lidar_initialized_(false),
  tf_odom_to_lidar_odom_(tf2::Transform::getIdentity()),
  has_previous_transform_(false),
  previous_transform_(tf2::Transform::getIdentity()),
  previous_time_(std::chrono::steady_clock::time_point::min()),
  smoothing_alpha_(0.3),
  smoothing_initialized_(false),
  filtered_transform_(tf2::Transform::getIdentity()),
  filtered_yaw_(0.0),
  velocity_smoothing_alpha_(0.15),
  velocity_smoothing_initialized_(false),
  filtered_linear_vel_x_(0.0),
  filtered_linear_vel_y_(0.0),
  filtered_angular_vel_z_(0.0)
{
  this->declare_parameter<std::string>("state_estimation_topic", "aft_mapped_to_init");
  this->declare_parameter<std::string>("registered_scan_topic", "cloud_registered");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("lidar_frame", "front_mid360");
  this->declare_parameter<std::string>("robot_base_frame", "gimbal_yaw");

  this->declare_parameter<double>("smoothing_alpha", 0.3);
  this->declare_parameter<double>("velocity_smoothing_alpha", 0.15);

  // Complementary filter parameters
  this->declare_parameter<bool>("cf_enabled", true);
  this->declare_parameter<double>("cf_tau", 15.0);
  this->declare_parameter<double>("cf_offset_gain", 0.0005);

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("smoothing_alpha", smoothing_alpha_);
  this->get_parameter("velocity_smoothing_alpha", velocity_smoothing_alpha_);
  this->get_parameter("cf_enabled", cf_enabled_);
  this->get_parameter("cf_tau", cf_tau_);
  this->get_parameter("cf_offset_gain", cf_offset_gain_);

  // Clamp alpha to valid range
  if (smoothing_alpha_ < 0.0) smoothing_alpha_ = 0.0;
  if (smoothing_alpha_ > 1.0) smoothing_alpha_ = 1.0;
  if (velocity_smoothing_alpha_ < 0.0) velocity_smoothing_alpha_ = 0.0;
  if (velocity_smoothing_alpha_ > 1.0) velocity_smoothing_alpha_ = 1.0;

  RCLCPP_INFO(this->get_logger(), "Odom TF smoothing alpha: %.2f", smoothing_alpha_);
  RCLCPP_INFO(this->get_logger(), "Velocity smoothing alpha: %.2f", velocity_smoothing_alpha_);
  RCLCPP_INFO(this->get_logger(),
              "Complementary filter: enabled=%s tau=%.1fs offset_gain=%.4f",
              cf_enabled_ ? "true" : "false", cf_tau_, cf_offset_gain_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  sensor_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);
  registered_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
  lidar_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);

  // Subscriptions for complementary filter
  chassis_attitude_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "serial/chassis_attitude", rclcpp::SensorDataQoS(),
    std::bind(&OdomBridgeNode::chassisAttitudeCallback, this, std::placeholders::_1));
  chassis_status_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "serial/chassis_status", rclcpp::SensorDataQoS(),
    std::bind(&OdomBridgeNode::chassisStatusCallback, this, std::placeholders::_1));

  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local();
  odom_to_lidar_odom_pub_ =
    this->create_publisher<geometry_msgs::msg::TransformStamped>("odom_to_lidar_odom", latched_qos);

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.depth = 5;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;

  odometry_sub_.subscribe(this, state_estimation_topic_, qos_profile);
  registered_scan_sub_.subscribe(this, registered_scan_topic_, qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, registered_scan_sub_);
  sync_->registerCallback(std::bind(
    &OdomBridgeNode::lidarOdometryAndPointCloudCallback, this,
    std::placeholders::_1, std::placeholders::_2));

  // Direct odometry subscription — bypasses ApproximateTime sync to ensure
  // TF and /odometry are published at the odometry rate, not gated by
  // cloud_registered availability.
  odometry_direct_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    state_estimation_topic_, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr & msg) {
      this->odometryCallback(msg);
    });

  // Publish an identity odom->base_footprint immediately so the TF tree is
  // complete from startup. Without this, Nav2 costmaps activate before
  // Point-LIO finishes IMU init and odom_bridge processes its first callback,
  // locking onto a timestamp where odom->base_footprint doesn't exist yet,
  // causing permanent "extrapolation into the past" errors.
  geometry_msgs::msg::TransformStamped bootstrap_tf;
  bootstrap_tf.header.stamp = this->get_clock()->now();
  bootstrap_tf.header.frame_id = odom_frame_;
  bootstrap_tf.child_frame_id = base_frame_;
  bootstrap_tf.transform.rotation.w = 1.0;  // identity quaternion
  tf_broadcaster_->sendTransform(bootstrap_tf);
  RCLCPP_INFO(this->get_logger(), "Published bootstrap odom->%s identity to complete TF tree",
              base_frame_.c_str());
}

void OdomBridgeNode::lidarOdometryAndPointCloudCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  if (!base_frame_to_lidar_initialized_) {
    try {
      const auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, odometry_msg->header.stamp, tf2::durationFromSec(0.05));
      tf2::Transform tf_base_frame_to_lidar;
      tf2::fromMsg(tf_stamped.transform, tf_base_frame_to_lidar);

      // Point-LIO first frame pose = rot_init (gravity alignment rotation).
      // lidar_odom frame is rotated by rot_init relative to the physical lidar frame at t=0.
      // Compensate: odom→lidar_odom = (base→lidar) * rot_init_inverse
      tf2::Transform tf_lidar_odom_to_lidar_t0;
      tf2::fromMsg(odometry_msg->pose.pose, tf_lidar_odom_to_lidar_t0);
      tf_odom_to_lidar_odom_ = tf_base_frame_to_lidar * tf_lidar_odom_to_lidar_t0.inverse();

      base_frame_to_lidar_initialized_ = true;

      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = odometry_msg->header.stamp;
      msg.header.frame_id = odom_frame_;
      msg.child_frame_id = "lidar_odom";
      msg.transform = tf2::toMsg(tf_odom_to_lidar_odom_);
      odom_to_lidar_odom_pub_->publish(msg);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Retrying...", ex.what());
      return;
    }
  }

  sensor_msgs::msg::PointCloud2 registered_scan_in_odom;
  pcl_ros::transformPointCloud(
    odom_frame_, tf_odom_to_lidar_odom_, *pcd_msg, registered_scan_in_odom);

  // Use the odometry paired with this scan. The latest high-rate odometry can
  // already be ahead of the cloud, causing motion-dependent obstacle offsets.
  tf2::Transform tf_lidar_odom_to_lidar;
  tf2::fromMsg(odometry_msg->pose.pose, tf_lidar_odom_to_lidar);
  const tf2::Transform tf_odom_to_lidar =
    tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar;

  sensor_msgs::msg::PointCloud2 sensor_scan;
  pcl_ros::transformPointCloud(
    lidar_frame_, tf_odom_to_lidar.inverse(), registered_scan_in_odom, sensor_scan);
  sensor_scan_pub_->publish(sensor_scan);

  registered_scan_pub_->publish(registered_scan_in_odom);
  {
    nav_msgs::msg::Odometry lidar_odom_out;
    lidar_odom_out.header.stamp = pcd_msg->header.stamp;
    lidar_odom_out.header.frame_id = odom_frame_;
    lidar_odom_out.child_frame_id = lidar_frame_;
    const auto & origin = tf_odom_to_lidar.getOrigin();
    lidar_odom_out.pose.pose.position.x = origin.x();
    lidar_odom_out.pose.pose.position.y = origin.y();
    lidar_odom_out.pose.pose.position.z = origin.z();
    lidar_odom_out.pose.pose.orientation = tf2::toMsg(tf_odom_to_lidar.getRotation());
    lidar_odometry_pub_->publish(lidar_odom_out);
  }
}

void OdomBridgeNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // --- One-time initialization (same logic as sync callback) ---
  if (!base_frame_to_lidar_initialized_) {
    try {
      const auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, msg->header.stamp, tf2::durationFromSec(0.05));
      tf2::Transform tf_base_frame_to_lidar;
      tf2::fromMsg(tf_stamped.transform, tf_base_frame_to_lidar);

      tf2::Transform tf_lidar_odom_to_lidar_t0;
      tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar_t0);
      tf_odom_to_lidar_odom_ = tf_base_frame_to_lidar * tf_lidar_odom_to_lidar_t0.inverse();

      base_frame_to_lidar_initialized_ = true;

      geometry_msgs::msg::TransformStamped odom_to_lidar_odom_msg;
      odom_to_lidar_odom_msg.header.stamp = msg->header.stamp;
      odom_to_lidar_odom_msg.header.frame_id = odom_frame_;
      odom_to_lidar_odom_msg.child_frame_id = "lidar_odom";
      odom_to_lidar_odom_msg.transform = tf2::toMsg(tf_odom_to_lidar_odom_);
      odom_to_lidar_odom_pub_->publish(odom_to_lidar_odom_msg);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Retrying...", ex.what());
      return;
    }
  }

  // --- Compute odom -> lidar from Point-LIO pose ---
  tf2::Transform tf_lidar_odom_to_lidar;
  tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar);
  tf2::Transform tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar;

  // Cache for point cloud callback (replaces sync-derived value)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_tf_odom_to_lidar_ = tf_odom_to_lidar;
    latest_odom_stamp_ = msg->header.stamp;
  }

  // --- Cancel lever-arm / gimbal rotation BEFORE EMA (Scheme B) ---
  // Performing the cancellation on the raw 3D lidar pose ensures R_static
  // (the LiDAR mount orientation, incl. ~pi/6 pitch) pairs perfectly with
  // (lidar->base)^-1 from the TF tree. The resulting odom->chassis is already
  // approximately 2D (roll≈0, pitch≈0, z≈0), so applying EMA afterwards — even
  // if it zeroes roll/pitch — cannot break the R_static pairing.
  const tf2::Transform tf_lidar_to_chassis =
    getTransform(lidar_frame_, base_frame_, msg->header.stamp);
  tf2::Transform tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;

  // --- EMA low-pass filter on the chassis pose ---
  // Chassis is horizontal (no R_static), so EMA on x/y/yaw is safe. Roll/pitch
  // are smoothed but will be zeroed by the 2D constraint below regardless.
  if (smoothing_alpha_ > 0.0) {
    const auto & origin = tf_odom_to_chassis.getOrigin();
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_odom_to_chassis.getRotation()).getRPY(roll, pitch, yaw);

    if (!smoothing_initialized_) {
      filtered_transform_ = tf_odom_to_chassis;
      filtered_roll_ = roll;
      filtered_pitch_ = pitch;
      filtered_yaw_ = yaw;
      smoothing_initialized_ = true;
    } else {
      const double a = smoothing_alpha_;
      const auto & fx = filtered_transform_.getOrigin().x();
      const auto & fy = filtered_transform_.getOrigin().y();
      const auto & fz = filtered_transform_.getOrigin().z();
      tf_odom_to_chassis.setOrigin(tf2::Vector3(
        a * origin.x() + (1.0 - a) * fx,
        a * origin.y() + (1.0 - a) * fy,
        a * origin.z() + (1.0 - a) * fz));

      filtered_roll_ += a * (roll - filtered_roll_);
      filtered_pitch_ += a * (pitch - filtered_pitch_);

      double dyaw = yaw - filtered_yaw_;
      while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
      while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
      filtered_yaw_ += a * dyaw;
      while (filtered_yaw_ > M_PI) filtered_yaw_ -= 2.0 * M_PI;
      while (filtered_yaw_ < -M_PI) filtered_yaw_ += 2.0 * M_PI;

      tf2::Quaternion q_f;
      q_f.setRPY(filtered_roll_, filtered_pitch_, filtered_yaw_);
      tf_odom_to_chassis.setRotation(q_f);
      filtered_transform_ = tf_odom_to_chassis;
    }
  }

  // Save EMA-smoothed chassis copy before 2D constraint / complementary filter.
  // /odometry (odom→gimbal_yaw) is intentionally left uncorrected — it
  // correctly represents gimbal_yaw's orientation including real gimbal
  // rotation. Derived from the smoothed chassis rather than the raw lidar
  // pose to keep the EMA jitter suppression on the odometry output.
  const tf2::Transform tf_odom_to_chassis_smoothed = tf_odom_to_chassis;

  // 2D constraint: z=0, roll=0, pitch=0
  // Complementary filter corrects only the chassis yaw (odom→base_footprint TF).
  // /odometry (odom→gimbal_yaw) uses the smoothed copy above, NOT this 2D / CF output.
  {
    const auto & origin = tf_odom_to_chassis.getOrigin();
    tf2::Quaternion q = tf_odom_to_chassis.getRotation();
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tf_odom_to_chassis.setOrigin(tf2::Vector3(origin.x(), origin.y(), 0.0));

    // --- Complementary filter: fuse MCU chassis_yaw (gimbal-immune, drifts)
    //     with Point-LIO derived yaw (gimbal-contaminated, no drift) ---
    double yaw_clean = yaw;
    if (cf_enabled_ && chassis_yaw_received_) {
      double yaw_mcu;
      int mode;
      {
        std::lock_guard<std::mutex> lock(chassis_state_mutex_);
        yaw_mcu = chassis_yaw_mcu_;
        mode = chassis_mode_;
      }

      // Align MCU IMU frame to odom frame via slowly-estimated offset
      if (!cf_offset_initialized_) {
        cf_yaw_offset_ = yaw - yaw_mcu;
        cf_offset_initialized_ = true;
      }
      double yaw_mcu_aligned = yaw_mcu + cf_yaw_offset_;

      // Clamp aligned MCU yaw to be near raw odom yaw (avoid ±2π jumps)
      yaw_mcu_aligned = yaw + normAngle(yaw_mcu_aligned - yaw);

      // Slowly update offset estimate — only in normal mode (not spin)
      if (mode == 0) {
        double offset_error = normAngle(yaw - (yaw_mcu + cf_yaw_offset_));
        cf_yaw_offset_ += cf_offset_gain_ * offset_error;
      }

      // Initialize bias
      if (!cf_yaw_initialized_) {
        cf_bias_ = 0.0;
        cf_yaw_initialized_ = true;
      }

      // Compute filter alpha from time constant + fixed dt estimate
      // (odom callback fires at ~100-200 Hz → dt ≈ 5-10 ms; tau=15s → α ≈ 0.0003-0.0007)
      constexpr double kDt = 0.01;  // conservative fixed step for stability
      double alpha = kDt / (cf_tau_ + kDt);

      // During spin modes, bypass filter (trust odom fully — chassis IS rotating fast)
      if (mode == 1 || mode == 2) {
        alpha = 0.0;
      }

      // Core: bias slowly tracks MCU→odom discrepancy
      double error = normAngle(yaw_mcu_aligned - yaw);
      cf_bias_ += alpha * normAngle(error - cf_bias_);
      yaw_clean = yaw + cf_bias_;
    }

    double chassis_yaw_clean = normAngle(yaw_clean);
    tf2::Quaternion q_2d;
    q_2d.setRPY(0.0, 0.0, chassis_yaw_clean);
    tf_odom_to_chassis.setRotation(q_2d);
  }

  // --- Rate-limited publish ---
  // Use steady_clock to avoid incompatible clock-source errors when
  // subtracting rclcpp::Time objects that may have different clock types.
  auto now = std::chrono::steady_clock::now();

  if (last_tf_publish_time_ == std::chrono::steady_clock::time_point{} ||
      std::chrono::duration<double>(now - last_tf_publish_time_).count() >= MIN_TF_PUBLISH_INTERVAL) {
    publishTransform(tf_odom_to_chassis, odom_frame_, base_frame_, msg->header.stamp);
    last_tf_publish_time_ = now;
  }

  if (last_odom_publish_time_ == std::chrono::steady_clock::time_point{} ||
      std::chrono::duration<double>(now - last_odom_publish_time_).count() >= MIN_ODOM_PUBLISH_INTERVAL) {
    const tf2::Transform tf_chassis_to_robot_base =
      getTransform(base_frame_, robot_base_frame_, msg->header.stamp);
    const tf2::Transform tf_odom_to_robot_base =
      tf_odom_to_chassis_smoothed * tf_chassis_to_robot_base;
    publishOdometry(tf_odom_to_robot_base, odom_frame_, robot_base_frame_, msg->header.stamp);
    last_odom_publish_time_ = now;
  }
}

tf2::Transform OdomBridgeNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    // Use the actual message timestamp so that dynamic transforms (e.g. gimbal yaw
    // joint) are queried at the same time as the Point-LIO odometry. This ensures the
    // gimbal rotation cancels out when computing odom->base_footprint:
    //   odom->base = (odom->lidar at T) * (base->lidar at T)^-1
    // A small tolerance is needed because robot_state_publisher publishes URDF
    // transforms slightly behind the high-frequency Point-LIO timestamps.
    const auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, tf2::durationFromSec(0.05));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (const tf2::ExtrapolationException & ex) {
    // Fallback to latest available transform if the requested time is too new
    try {
      const auto transform_stamped = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
      tf2::Transform transform;
      tf2::fromMsg(transform_stamped.transform, transform);
      return transform;
    } catch (tf2::TransformException & ex2) {
      RCLCPP_WARN(this->get_logger(), "TF lookup fallback failed: %s. Returning identity.", ex2.what());
      return tf2::Transform::getIdentity();
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Returning identity.", ex.what());
    return tf2::Transform::getIdentity();
  }
}

void OdomBridgeNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  tf_broadcaster_->sendTransform(transform_msg);
}

void OdomBridgeNode::publishOdometry(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  if (has_previous_transform_) {
    const auto current_time = std::chrono::steady_clock::now();
    const double dt =
      std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - previous_time_).count() *
      1e-9;

    if (dt > 0.0) {
      const auto linear_velocity = (transform.getOrigin() - previous_transform_.getOrigin()) / dt;
      const tf2::Quaternion q_diff =
        transform.getRotation() * previous_transform_.getRotation().inverse();
      const auto angular_velocity = q_diff.getAxis() * q_diff.getAngle() / dt;

      // --- EMA low-pass filter on velocity (suppresses finite-difference noise) ---
      double vx = linear_velocity.x();
      double vy = linear_velocity.y();
      double vz = angular_velocity.z();
      if (velocity_smoothing_alpha_ > 0.0) {
        const double a = velocity_smoothing_alpha_;
        if (!velocity_smoothing_initialized_) {
          filtered_linear_vel_x_ = vx;
          filtered_linear_vel_y_ = vy;
          filtered_angular_vel_z_ = vz;
          velocity_smoothing_initialized_ = true;
        } else {
          filtered_linear_vel_x_ = a * vx + (1.0 - a) * filtered_linear_vel_x_;
          filtered_linear_vel_y_ = a * vy + (1.0 - a) * filtered_linear_vel_y_;
          filtered_angular_vel_z_ = a * vz + (1.0 - a) * filtered_angular_vel_z_;
        }
        vx = filtered_linear_vel_x_;
        vy = filtered_linear_vel_y_;
        vz = filtered_angular_vel_z_;
      }
      // --- End velocity EMA filter ---

      out.twist.twist.linear.x = vx;
      out.twist.twist.linear.y = vy;
      out.twist.twist.linear.z = 0.0;
      out.twist.twist.angular.x = 0.0;
      out.twist.twist.angular.y = 0.0;
      out.twist.twist.angular.z = vz;
    }

    previous_transform_ = transform;
    previous_time_ = current_time;
  } else {
    previous_transform_ = transform;
    previous_time_ = std::chrono::steady_clock::now();
    has_previous_transform_ = true;
  }

  odometry_pub_->publish(out);
}

void OdomBridgeNode::chassisAttitudeCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Extract chassis_yaw from serial/chassis_attitude
  // msg->name = {"chassis_pitch", "chassis_yaw"}
  auto it_yaw = std::find(msg->name.begin(), msg->name.end(), "chassis_yaw");
  if (it_yaw == msg->name.end()) {
    return;
  }
  auto idx = static_cast<size_t>(std::distance(msg->name.begin(), it_yaw));
  if (idx >= msg->position.size()) {
    return;
  }

  std::lock_guard<std::mutex> lock(chassis_state_mutex_);
  chassis_yaw_mcu_ = static_cast<double>(msg->position[idx]);
  chassis_yaw_received_ = true;
}

void OdomBridgeNode::chassisStatusCallback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  // serial/chassis_status = [chassis_power, chassis_mode]
  if (msg->data.size() < 2) {
    return;
  }

  std::lock_guard<std::mutex> lock(chassis_state_mutex_);
  chassis_mode_ = static_cast<int>(msg->data[1]);
}

}  // namespace odom_bridge

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(odom_bridge::OdomBridgeNode)
