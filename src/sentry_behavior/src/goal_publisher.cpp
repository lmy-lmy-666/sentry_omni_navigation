// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sentry_behavior/goal_publisher.hpp"

#include <cmath>

namespace sentry_behavior
{

GoalPublisher::GoalPublisher(rclcpp::Node * node, const std::string & topic)
: node_(node)
{
  pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 1);
}

void GoalPublisher::request(const GoalCmd & goal)
{
  // 去重: 精确 (x,y) 相等 (坐标为字面量, 不引入 epsilon 以保持与旧实现一致).
  if (last_delivered_ && last_delivered_->first == goal.x &&
    last_delivered_->second == goal.y)
  {
    return;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = goal.x;
  msg.pose.position.y = goal.y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = std::sin(goal.yaw * 0.5);
  msg.pose.orientation.w = std::cos(goal.yaw * 0.5);

  const size_t subs = pub_->get_subscription_count();
  pub_->publish(msg);

  if (subs > 0) {
    last_delivered_ = std::make_pair(goal.x, goal.y);
    RCLCPP_INFO(node_->get_logger(), "goal -> (%.2f, %.2f)", goal.x, goal.y);
  } else {
    // 无订阅者: 不 latch, 下个 tick 还会重发, 直到 nav2 订阅上.
    RCLCPP_WARN(
      node_->get_logger(),
      "goal (%.2f, %.2f) published but 0 subscribers, will retry next tick", goal.x, goal.y);
  }
}

void GoalPublisher::reset()
{
  last_delivered_.reset();
}

}  // namespace sentry_behavior
