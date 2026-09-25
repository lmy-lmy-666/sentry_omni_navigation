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

#ifndef SENTRY_BEHAVIOR__GOAL_PUBLISHER_HPP_
#define SENTRY_BEHAVIOR__GOAL_PUBLISHER_HPP_

#include <optional>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sentry_behavior
{

// 一个战术目标点 (惯性系 map 下).
struct GoalCmd
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// 单一 /goal_pose publisher 封装, 1:1 复刻旧 PubGoal 语义 (见方案 §6):
//   - 去重: 与上次"已送达"坐标精确 (x,y) 相等则不重发;
//   - 即使 0 订阅者也 publish (兼容 nav2 bt_navigator 晚于本节点就绪);
//   - 仅当 publish 时有订阅者才记为"已送达"(latch), 否则下个 tick 继续重发,
//     处理启动时序: 第一条若无人订阅会丢, 但不 latch 故会补发;
//   - reset() 清空已送达缓存 (进入 WAIT_START / 构造时调用), 使下一场重回
//     同坐标也能重新发布, 而非被陈旧去重屏蔽.
// publisher 在构造期一次性创建, 避免旧代码进程级共享 publisher 的 DDS 首包丢失.
class GoalPublisher
{
public:
  GoalPublisher(rclcpp::Node * node, const std::string & topic);

  // 每 tick 调用 (level-trigger). 内部按上述语义去重 / 重发.
  void request(const GoalCmd & goal);

  // 清空已送达缓存.
  void reset();

private:
  rclcpp::Node * node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  std::optional<std::pair<double, double>> last_delivered_;
};

}  // namespace sentry_behavior

#endif  // SENTRY_BEHAVIOR__GOAL_PUBLISHER_HPP_
