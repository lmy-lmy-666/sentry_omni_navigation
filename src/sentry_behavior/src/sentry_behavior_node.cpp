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

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "sentry_behavior/behavior_machine.hpp"
#include "sentry_behavior/goal_publisher.hpp"
#include "sentry_behavior/referee_snapshot.hpp"
#include "sentry_behavior/strategies.hpp"
#include "sentry_behavior/viz/state_viz_server.hpp"

namespace sentry_behavior
{
namespace
{

std::string build_graph_json(const std::string & strat, const std::vector<std::string> & tac_ids)
{
  std::string s = "{\"type\":\"graph\",\"v\":1,\"strategy\":\"" + strat +
    "\",\"states\":[{\"id\":\"WAIT_START\",\"parent\":null},{\"id\":\"IN_MATCH\",\"parent\":null}";
  for (const auto & id : tac_ids) {
    s += ",{\"id\":\"" + id + "\",\"parent\":\"IN_MATCH\"}";
  }
  s += "],\"transitions\":[{\"from\":\"WAIT_START\",\"to\":\"IN_MATCH\",\"label\":\"progress==4\"},"
    "{\"from\":\"IN_MATCH\",\"to\":\"WAIT_START\",\"label\":\"progress!=4\"}]}";
  return s;
}

std::string build_state_json(
  int64_t t_ms, const std::vector<std::string> & path,
  const std::optional<GoalCmd> & goal, const RefereeSnapshot & r)
{
  std::string s = "{\"type\":\"state\",\"t\":" + std::to_string(t_ms) + ",\"active\":[";
  for (size_t i = 0; i < path.size(); ++i) {
    if (i) {s += ",";}
    s += "\"" + path[i] + "\"";
  }
  s += "],\"goal\":";
  if (goal) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "{\"x\":%.2f,\"y\":%.2f}", goal->x, goal->y);
    s += buf;
  } else {
    s += "null";
  }
  s += ",\"referee\":{\"progress\":" + std::to_string(static_cast<int>(r.progress)) +
    ",\"remain\":" + std::to_string(r.remain) +
    ",\"hp\":" + std::to_string(r.hp) +
    ",\"ammo\":" + std::to_string(r.ammo) +
    ",\"outpost_hp\":" + std::to_string(r.outpost_hp) + "}}";
  return s;
}

std::string build_transition_json(int64_t t_ms, const std::string & from, const std::string & to)
{
  return "{\"type\":\"transition\",\"t\":" + std::to_string(t_ms) +
         ",\"from\":\"" + from + "\",\"to\":\"" + to + "\"}";
}

}  // namespace

class SentryBehaviorNode : public rclcpp::Node
{
public:
  explicit SentryBehaviorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("sentry_behavior_node", options)
  {
    const std::string strategy_name = declare_parameter<std::string>("strategy", "rmuc_defend");
    const double tick_frequency = declare_parameter<double>("tick_frequency", 20.0);
    const bool viz_enable = declare_parameter<bool>("viz_enable", true);
    const int viz_port = declare_parameter<int>("viz_port", 1667);

    StrategyParams params;
    params.rmuc_hp_min = declare_parameter<int>("rmuc_hp_min", params.rmuc_hp_min);
    params.attack_hp_min = declare_parameter<int>("attack_hp_min", params.attack_hp_min);
    params.patrol_hp_min = declare_parameter<int>("patrol_hp_min", params.patrol_hp_min);
    params.ammo_min = declare_parameter<int>("ammo_min", params.ammo_min);
    params.outpost_hp_min = declare_parameter<int>("outpost_hp_min", params.outpost_hp_min);

    if (tick_frequency <= 0.0) {
      throw std::runtime_error("tick_frequency must be > 0");
    }

    Strategy strategy = make_strategy(strategy_name, params);

    goal_pub_ = std::make_unique<GoalPublisher>(this, "/goal_pose");
    machine_ = std::make_unique<BehaviorMachine>(std::move(strategy), goal_pub_.get());

    if (viz_enable) {
      viz_server_ = std::make_unique<StateVizServer>(
        static_cast<uint16_t>(viz_port),
        build_graph_json(strategy_name, machine_->tactical_state_ids()));
    }

    const auto qos = rclcpp::QoS(10);
    game_sub_ = create_subscription<rm_interfaces::msg::GameStatus>(
      "referee/game_status", qos,
      [this](const rm_interfaces::msg::GameStatus::SharedPtr msg) {
        snapshot_.progress = msg->game_progress;
        snapshot_.remain = msg->stage_remain_time;
        snapshot_.game_valid = true;
      });
    robot_sub_ = create_subscription<rm_interfaces::msg::RobotStatus>(
      "referee/robot_status", qos,
      [this](const rm_interfaces::msg::RobotStatus::SharedPtr msg) {
        snapshot_.hp = msg->current_hp;
        snapshot_.ammo = msg->projectile_allowance_17mm;
        snapshot_.robot_valid = true;
      });
    hp_sub_ = create_subscription<rm_interfaces::msg::GameRobotHP>(
      "referee/all_robot_hp", qos,
      [this](const rm_interfaces::msg::GameRobotHP::SharedPtr msg) {
        snapshot_.outpost_hp = msg->ally_outpost_hp;
        snapshot_.outpost_valid = true;
      });

    const auto period = std::chrono::duration<double>(1.0 / tick_frequency);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        Ctx ctx{snapshot_, now()};
        machine_->tick(ctx);
        if (viz_server_) {
          const int64_t t_ms = now().nanoseconds() / 1000000;
          auto path = machine_->active_path();
          const std::string leaf = path.empty() ? std::string("ROOT") : path.back();
          if (leaf != prev_leaf_) {
            viz_server_->push_transition(build_transition_json(t_ms, prev_leaf_, leaf));
            prev_leaf_ = leaf;
          }
          viz_server_->update_state(
            build_state_json(t_ms, path, machine_->current_goal(), snapshot_));
        }
      });

    RCLCPP_INFO(
      get_logger(), "sentry_behavior_node ready, strategy=%s", strategy_name.c_str());
  }

private:
  RefereeSnapshot snapshot_;
  std::unique_ptr<GoalPublisher> goal_pub_;
  std::unique_ptr<BehaviorMachine> machine_;
  std::unique_ptr<StateVizServer> viz_server_;
  std::string prev_leaf_{"ROOT"};
  rclcpp::Subscription<rm_interfaces::msg::GameStatus>::SharedPtr game_sub_;
  rclcpp::Subscription<rm_interfaces::msg::RobotStatus>::SharedPtr robot_sub_;
  rclcpp::Subscription<rm_interfaces::msg::GameRobotHP>::SharedPtr hp_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sentry_behavior::SentryBehaviorNode>(rclcpp::NodeOptions());
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("sentry_behavior_node"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
