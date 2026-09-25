// Copyright 2026 Boombroke
//
// ROS 2 node wrapper — subscribes to referee data + odometry,
// drives the simplified FSM, publishes Nav2 goals.
//
#ifndef OMNI_DECISION_SAMPLE__DECISION_NODE_HPP_
#define OMNI_DECISION_SAMPLE__DECISION_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "omni_decision_sample/arrival_tracker.hpp"
#include "omni_decision_sample/context.hpp"
#include "omni_decision_sample/fsm.hpp"

namespace omni_decision_sample
{

class DecisionNode : public rclcpp::Node
{
public:
  explicit DecisionNode(const rclcpp::NodeOptions & options);

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // --- timer -------------------------------------------------------
  void on_tick();

  // --- navigation --------------------------------------------------
  void publish_goal(const Waypoint & wp);
  void publish_goal_topic_fallback(const Waypoint & wp);
  void cancel_nav();

  // --- bump traverse (open-loop chassis velocity, bypassing Nav2) ---
  /// Publish linear.x only (y=0, yaw=0) to the chassis cmd_vel topic. vx signed.
  void publish_bump_vel(double vx);

  // --- Nav2 action callbacks ---------------------------------------
  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

  // --- subscriptions -----------------------------------------------
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ==================================================================
  //  members
  // ==================================================================

  Context ctx_;
  std::unique_ptr<DecisionFsm> fsm_;

  State last_logged_state_{State::IDLE};

  // --- navigation state --------------------------------------------
  bool nav_goals_active_{false};
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;

  // Per-goal arrival resolver (ROS-free, unit-tested in arrival_tracker_test).
  // Latches ARRIVED so late Nav2 feedback can't demote it back to MOVING.
  // Constructed in the initializer list from goal_reached_distance_tolerance_.
  ArrivalTracker arrival_;

  // --- fallback (Nav2 unavailable → PoseStamped + odom check) ------
  double fallback_goal_x_{0.0};
  double fallback_goal_y_{0.0};
  bool   fallback_goal_active_{false};
  double current_x_{0.0};
  double current_y_{0.0};

  // --- ROS handles -------------------------------------------------
  rclcpp::Subscription<rm_interfaces::msg::GameStatus>::SharedPtr   sub_game_status_;
  rclcpp::Subscription<rm_interfaces::msg::RobotStatus>::SharedPtr   sub_robot_status_;
  rclcpp::Subscription<rm_interfaces::msg::RfidStatus>::SharedPtr    sub_rfid_status_;
  rclcpp::Subscription<rm_interfaces::msg::GameRobotHP>::SharedPtr   sub_robot_hp_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           sub_odom_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // --- TF (odom-frame odometry → map-frame for fallback arrival check) ---
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- parameters --------------------------------------------------
  std::string goal_frame_{"map"};
  std::string nav_action_name_{"navigate_to_pose"};
  double      goal_reached_distance_tolerance_{0.25};
  std::string bump_cmd_vel_topic_{"cmd_vel_chassis"};
};

}  // namespace omni_decision_sample

#endif  // OMNI_DECISION_SAMPLE__DECISION_NODE_HPP_
