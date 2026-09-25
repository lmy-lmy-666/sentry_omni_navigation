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

#include "sentry_behavior/strategies.hpp"

#include <stdexcept>

namespace sentry_behavior
{
namespace
{

Strategy make_rmuc_defend(const StrategyParams & p)
{
  const int hp_min = p.rmuc_hp_min;
  const int ammo_min = p.ammo_min;
  Strategy s;
  s.name = "rmuc_defend";
  s.states.push_back(
    TacticalState{
      "DEFEND",
      [hp_min, ammo_min](const Ctx & c) {
        return c.referee.robot_valid && c.referee.hp >= hp_min && c.referee.ammo >= ammo_min;
      },
      GoalCmd{3.71, -0.61, 0.0}});
  s.states.push_back(
    TacticalState{
      "SUPPLY",
      [](const Ctx &) {return true;},
      GoalCmd{-0.27, -3.94, 0.0}});
  return s;
}

Strategy make_attack(const StrategyParams & p)
{
  const int hp_min = p.attack_hp_min;
  const int ammo_min = p.ammo_min;
  Strategy s;
  s.name = "a";
  s.states.push_back(
    TacticalState{
      "ATTACK",
      [hp_min, ammo_min](const Ctx & c) {
        return c.referee.robot_valid && c.referee.hp >= hp_min && c.referee.ammo >= ammo_min;
      },
      GoalCmd{3.71, -0.61, 0.0}});
  s.states.push_back(
    TacticalState{
      "SUPPLY",
      [](const Ctx &) {return true;},
      GoalCmd{-0.27, -3.94, 0.0}});
  return s;
}

Strategy make_patrol(const StrategyParams & p)
{
  const int hp_min = p.patrol_hp_min;
  const int ammo_min = p.ammo_min;
  const int outpost_min = p.outpost_hp_min;
  Strategy s;
  s.name = "b";
  s.states.push_back(
    TacticalState{
      "PATROL",
      [hp_min, ammo_min, outpost_min](const Ctx & c) {
        return c.referee.outpost_valid && c.referee.outpost_hp >= outpost_min &&
               c.referee.robot_valid && c.referee.hp >= hp_min && c.referee.ammo >= ammo_min;
      },
      GoalCmd{9.17, 4.07, 0.0}});
  s.states.push_back(
    TacticalState{
      "SUPPLY",
      [outpost_min](const Ctx & c) {
        return c.referee.outpost_valid && c.referee.outpost_hp >= outpost_min;
      },
      GoalCmd{-0.27, -3.94, 0.0}});
  s.states.push_back(
    TacticalState{
      "FALLBACK",
      [](const Ctx &) {return true;},
      GoalCmd{3.27, -0.90, 0.0}});
  return s;
}

}  // namespace

Strategy make_strategy(const std::string & name, const StrategyParams & p)
{
  if (name == "rmuc_defend") {
    return make_rmuc_defend(p);
  }
  if (name == "a") {
    return make_attack(p);
  }
  if (name == "b") {
    return make_patrol(p);
  }
  throw std::runtime_error("unknown strategy: " + name);
}

}  // namespace sentry_behavior
