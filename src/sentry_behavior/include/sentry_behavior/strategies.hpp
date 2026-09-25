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

#ifndef SENTRY_BEHAVIOR__STRATEGIES_HPP_
#define SENTRY_BEHAVIOR__STRATEGIES_HPP_

#include <string>

#include "sentry_behavior/behavior_machine.hpp"

namespace sentry_behavior
{

// 可调阈值, 由节点参数注入. 默认值 = 各战术树原"生效"语义 (方案 §2/§7,
// 已丢弃 RMUC/b 中不可达的 400/50、381/50 死阈值).
struct StrategyParams
{
  int rmuc_hp_min = 151;
  int attack_hp_min = 300;
  int patrol_hp_min = 300;
  int ammo_min = 1;
  int outpost_hp_min = 1;
};

// 按名构建策略; 未知名抛 std::runtime_error.
Strategy make_strategy(const std::string & name, const StrategyParams & p);

}  // namespace sentry_behavior

#endif  // SENTRY_BEHAVIOR__STRATEGIES_HPP_
