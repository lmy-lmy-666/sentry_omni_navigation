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

#ifndef SENTRY_BEHAVIOR__VIZ__STATE_VIZ_SERVER_HPP_
#define SENTRY_BEHAVIOR__VIZ__STATE_VIZ_SERVER_HPP_

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sentry_behavior
{

// 嵌入式 TCP 状态可视化协议服务端 (NDJSON, 逐行 JSON), 供独立 Rust/C GUI 消费 (无 ROS 依赖).
// 独立线程 accept + 发送; 写为非阻塞 + 慢客户端丢帧, 保证永不阻塞 20Hz 决策 tick.
// 客户端连接即收 graph 帧 (静态拓扑) + 当前 state 帧, 之后持续收 state / transition.
class StateVizServer
{
public:
  StateVizServer(uint16_t port, std::string graph_json);
  ~StateVizServer();

  StateVizServer(const StateVizServer &) = delete;
  StateVizServer & operator=(const StateVizServer &) = delete;

  void update_state(const std::string & state_json);
  void push_transition(const std::string & transition_json);

private:
  void run();
  void accept_clients();
  void broadcast(const std::string & line);

  std::string graph_json_;
  int listen_fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread thread_;

  std::mutex mtx_;                      // 仅保护 latest_state_ / transitions_ (跨线程共享)
  std::string latest_state_;
  std::deque<std::string> transitions_;

  std::vector<int> clients_;            // 仅 viz 线程访问, 无需加锁
};

}  // namespace sentry_behavior

#endif  // SENTRY_BEHAVIOR__VIZ__STATE_VIZ_SERVER_HPP_
