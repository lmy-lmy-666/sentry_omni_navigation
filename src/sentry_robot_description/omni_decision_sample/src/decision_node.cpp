// Copyright 2026 Boombroke
//
// ROS 2 node — bridges referee/odom data → Context → FSM → Nav2 goals.
//
#include "omni_decision_sample/decision_node.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace omni_decision_sample
{

DecisionNode::DecisionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("omni_decision_sample", options)
{
  // --- parameters --------------------------------------------------
  const std::string profile_path = this->declare_parameter<std::string>("profile_path", "");
  const double tick_hz = this->declare_parameter<double>("tick_frequency", 10.0);
  const std::string goal_topic = this->declare_parameter<std::string>("goal_topic", "/goal_pose");
  nav_action_name_ = this->declare_parameter<std::string>("nav_action_name", "navigate_to_pose");
  goal_reached_distance_tolerance_ =
    this->declare_parameter<double>("goal_reached_distance_tolerance", 0.25);
  arrival_.set_tolerance(goal_reached_distance_tolerance_);
  goal_frame_ = this->declare_parameter<std::string>("goal_frame", "map");
  bump_cmd_vel_topic_ =
    this->declare_parameter<std::string>("bump_cmd_vel_topic", "cmd_vel_chassis");

  if (profile_path.empty()) {
    throw std::runtime_error("omni_decision_sample: 'profile_path' parameter is required");
  }

  // --- profile -----------------------------------------------------
  Profile profile = load_profile(profile_path);
  ctx_.set_thresholds(profile.thresholds);
  RCLCPP_INFO(
    get_logger(), "loaded profile '%s' (%zu patrol waypoints)",
    profile.name.c_str(), profile.patrol.size());

  // --- publishers --------------------------------------------------
  pub_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
  // Open-loop chassis velocity while crossing undulating terrain. Depth 1: only
  // the latest command matters; we bypass Nav2 and drive the chassis directly.
  pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>(bump_cmd_vel_topic_, 1);

  // --- Nav2 action client ------------------------------------------
  nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav_action_name_);

  // --- TF (transform odometry from its odom frame into goal_frame) --
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // --- FSM ---------------------------------------------------------
  fsm_ = std::make_unique<DecisionFsm>(
    std::move(profile),
    [this](const Waypoint & wp) { publish_goal(wp); },
    [this]() { cancel_nav(); },
    [this](double vx) { publish_bump_vel(vx); });

  // --- subscriptions -----------------------------------------------
  sub_game_status_ = create_subscription<rm_interfaces::msg::GameStatus>(
    "referee/game_status", 10,
    [this](const rm_interfaces::msg::GameStatus::SharedPtr m) { ctx_.update(*m); });
  sub_robot_status_ = create_subscription<rm_interfaces::msg::RobotStatus>(
    "referee/robot_status", 10,
    [this](const rm_interfaces::msg::RobotStatus::SharedPtr m) { ctx_.update(*m); });
  sub_rfid_status_ = create_subscription<rm_interfaces::msg::RfidStatus>(
    "referee/rfidStatus", 10,
    [this](const rm_interfaces::msg::RfidStatus::SharedPtr m) { ctx_.update(*m); });
  sub_robot_hp_ = create_subscription<rm_interfaces::msg::GameRobotHP>(
    "referee/all_robot_hp", 10,
    [this](const rm_interfaces::msg::GameRobotHP::SharedPtr m) { ctx_.update(*m); });
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry", 10,
    std::bind(&DecisionNode::odometry_callback, this, std::placeholders::_1));

  // --- timer -------------------------------------------------------
  const auto period = std::chrono::duration<double>(1.0 / tick_hz);
  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { on_tick(); });

  RCLCPP_INFO(get_logger(), "omni_decision_sample started @ %.1f Hz", tick_hz);
}

// ============================================================================
//  on_tick — main loop
// ============================================================================

void DecisionNode::on_tick()
{
  const double now_s = this->now().seconds();
  ctx_.set_now(now_s);

  // Fallback odom-based arrival detection (Nav2 unavailable)
  if (fallback_goal_active_ && nav_goals_active_ && !arrival_.latched()) {
    const double dx = current_x_ - fallback_goal_x_;
    const double dy = current_y_ - fallback_goal_y_;
    arrival_.update_fallback_distance(std::hypot(dx, dy));
    if (arrival_.arrived()) {
      ctx_.set_nav_status(NavStatus::ARRIVED);
      fallback_goal_active_ = false;
    }
  }

  if (!ctx_.has_referee()) return;

  fsm_->tick(ctx_, now_s);

  const State current = fsm_->state();

  // Clear nav state when entering IDLE
  if (current == State::IDLE && nav_goals_active_) {
    nav_goals_active_ = false;
    fallback_goal_active_ = false;
  }

  // Log state transitions
  if (current != last_logged_state_) {
    RCLCPP_INFO(
      get_logger(), "state -> %-8s | reason: %-35s | hp=%3u/%-3u ammo=%3u remain=%3ds nav=%d",
      to_string(current), fsm_->state_reason(),
      ctx_.hp(), ctx_.max_hp(), ctx_.ammo(), ctx_.remain_time(),
      static_cast<int>(ctx_.nav_status()));
    last_logged_state_ = current;
  }
}

// ============================================================================
//  publish_goal — Nav2 action client (with fallback to PoseStamped topic)
// ============================================================================

void DecisionNode::publish_goal(const Waypoint & wp)
{
  fallback_goal_active_ = false;  // clear old fallback coordinates
  arrival_.reset();               // new goal → fresh arrival latch

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.header.frame_id = goal_frame_;
  goal_msg.pose.pose.position.x = wp.x;
  goal_msg.pose.pose.position.y = wp.y;
  goal_msg.pose.pose.position.z = 0.0;
  goal_msg.pose.pose.orientation.w = 1.0;

  if (!nav_action_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Nav2 action '%s' not ready; falling back to PoseStamped topic '%s'",
      nav_action_name_.c_str(), pub_goal_->get_topic_name());
    publish_goal_topic_fallback(wp);
    return;
  }

  // Preempt any in-flight goal cleanly. We cancel the old goal on the Nav2
  // side, and — crucially — drop our handle to it NOW. Between async_send_goal
  // and the new goal's response callback, current_goal_handle_ stays null, so
  // the old goal's ABORTED/CANCELED result falls through the null-guard in
  // result_callback instead of being mistaken for the new goal failing.
  if (current_goal_handle_) {
    (void)nav_action_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_.reset();
  }

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.goal_response_callback =
    [this](const GoalHandleNavigateToPose::SharedPtr & gh) { goal_response_callback(gh); };
  options.feedback_callback =
    [this](GoalHandleNavigateToPose::SharedPtr gh,
           const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      feedback_callback(gh, fb);
    };
  options.result_callback =
    [this](const GoalHandleNavigateToPose::WrappedResult & r) { result_callback(r); };

  (void)nav_action_client_->async_send_goal(goal_msg, options);
  nav_goals_active_ = true;
  ctx_.set_nav_status(NavStatus::MOVING);
  RCLCPP_DEBUG(get_logger(), "goal -> (%.2f, %.2f)", wp.x, wp.y);
}

void DecisionNode::publish_goal_topic_fallback(const Waypoint & wp)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = this->now();
  goal.header.frame_id = goal_frame_;
  goal.pose.position.x = wp.x;
  goal.pose.position.y = wp.y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.w = 1.0;
  pub_goal_->publish(goal);
  nav_goals_active_ = true;
  fallback_goal_x_ = wp.x;
  fallback_goal_y_ = wp.y;
  fallback_goal_active_ = true;
  ctx_.set_nav_status(NavStatus::MOVING);
  RCLCPP_DEBUG(get_logger(), "fallback goal -> (%.2f, %.2f)", wp.x, wp.y);
}

// ============================================================================
//  cancel_nav
// ============================================================================

void DecisionNode::cancel_nav()
{
  if (!nav_goals_active_) return;

  if (!nav_action_client_->action_server_is_ready()) {
    nav_goals_active_ = false;
    fallback_goal_active_ = false;
    arrival_.on_canceled();
    ctx_.set_nav_status(NavStatus::IDLE);
    return;
  }

  (void)nav_action_client_->async_cancel_all_goals();
  current_goal_handle_.reset();
  nav_goals_active_ = false;
  arrival_.on_canceled();
  ctx_.set_nav_status(NavStatus::IDLE);
  RCLCPP_INFO(get_logger(), "cancelled all Nav2 goals");
}

// ============================================================================
//  publish_bump_vel — open-loop chassis velocity (bump traverse only)
// ============================================================================

void DecisionNode::publish_bump_vel(double vx)
{
  // Swerve constraint: pure straight-line X translation. y and yaw are always
  // zero so all four wheels point along the travel direction. Published to the
  // chassis topic directly, bypassing Nav2 and fake_vel_transform (no spin).
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx;
  cmd.linear.y = 0.0;
  cmd.angular.z = 0.0;
  pub_cmd_vel_->publish(cmd);
}

// ============================================================================
//  Nav2 action callbacks
// ============================================================================

void DecisionNode::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    arrival_.on_result_failed();
    ctx_.set_nav_status(arrival_.status());
    nav_goals_active_ = false;
    current_goal_handle_.reset();
    RCLCPP_WARN(get_logger(), "Nav2 rejected the goal");
    return;
  }
  current_goal_handle_ = goal_handle;
  ctx_.set_nav_status(NavStatus::MOVING);
}

void DecisionNode::feedback_callback(
  GoalHandleNavigateToPose::SharedPtr goal_handle,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  if (!current_goal_handle_ ||
      goal_handle->get_goal_id() != current_goal_handle_->get_goal_id()) {
    return;  // stale goal
  }
  // The tracker latches ARRIVED, so a later feedback with a larger
  // distance_remaining can't demote it back to MOVING (see ArrivalTracker).
  arrival_.update_feedback(feedback->distance_remaining);
  ctx_.set_nav_status(arrival_.status());
}

void DecisionNode::result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
  if (!current_goal_handle_ || result.goal_id != current_goal_handle_->get_goal_id()) {
    RCLCPP_DEBUG(get_logger(), "ignoring stale Nav2 result");
    return;
  }
  current_goal_handle_.reset();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      arrival_.on_result_succeeded();   // latch against any late feedback
      ctx_.set_nav_status(arrival_.status());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      arrival_.on_canceled();
      ctx_.set_nav_status(arrival_.status());
      nav_goals_active_ = false;
      break;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::UNKNOWN:
      arrival_.on_result_failed();
      ctx_.set_nav_status(arrival_.status());
      nav_goals_active_ = false;
      break;
  }
}

// ============================================================================
//  odometry callback
// ============================================================================

void DecisionNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Odometry is in the odom frame; goals are in goal_frame_ (map). The fallback
  // arrival check compares position against the goal, so transform the position
  // into goal_frame_ first — otherwise the map→odom drift corrupts the distance.
  geometry_msgs::msg::PointStamped p_in;
  p_in.header = msg->header;                     // frame_id = odom (from odom_bridge)
  p_in.point = msg->pose.pose.position;

  const std::string & src_frame = msg->header.frame_id;

  if (src_frame.empty() || src_frame == goal_frame_) {
    // already in goal frame (or unknown) — use as-is
    current_x_ = p_in.point.x;
    current_y_ = p_in.point.y;
  } else {
    try {
      geometry_msgs::msg::PointStamped p_out =
        tf_buffer_->transform(p_in, goal_frame_, tf2::durationFromSec(0.1));
      current_x_ = p_out.point.x;
      current_y_ = p_out.point.y;
    } catch (const tf2::TransformException & ex) {
      // TF unavailable (e.g. localization not up yet). Fall back to raw odom
      // position — the fallback arrival check may be off until TF is available,
      // but the primary Nav2 feedback path (used when Nav2 is running) is
      // unaffected, and we avoid crashing / stalling.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s->%s unavailable (%s); using raw odom for fallback arrival check",
        src_frame.c_str(), goal_frame_.c_str(), ex.what());
      current_x_ = p_in.point.x;
      current_y_ = p_in.point.y;
    }
  }

  ctx_.set_sentry_position(current_x_, current_y_);
}

}  // namespace omni_decision_sample

RCLCPP_COMPONENTS_REGISTER_NODE(omni_decision_sample::DecisionNode)
