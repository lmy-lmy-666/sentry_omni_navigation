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

#ifndef SENTRY_BEHAVIOR__REFEREE_SNAPSHOT_HPP_
#define SENTRY_BEHAVIOR__REFEREE_SNAPSHOT_HPP_

#include <cstdint>

namespace sentry_behavior
{

// 裁判系统最新值快照. 单线程 executor 下由订阅回调写、timer 读, 无需锁.
// 各 *_valid 在收到首条对应消息后置 true; guard 在数据缺失时按 false 处理,
// 复刻旧 SimpleConditionNode "缺输入返回 FAILURE" 的语义.
struct RefereeSnapshot
{
  // referee/game_status (GameStatus)
  bool game_valid = false;
  uint8_t progress = 0;     // game_progress: 4=RUNNING, 5=GAME_OVER
  int32_t remain = 0;       // stage_remain_time (s)

  // referee/robot_status (RobotStatus)
  bool robot_valid = false;
  uint16_t hp = 0;          // current_hp
  uint16_t ammo = 0;        // projectile_allowance_17mm

  // referee/all_robot_hp (GameRobotHP)
  bool outpost_valid = false;
  uint16_t outpost_hp = 0;  // ally_outpost_hp
};

}  // namespace sentry_behavior

#endif  // SENTRY_BEHAVIOR__REFEREE_SNAPSHOT_HPP_
