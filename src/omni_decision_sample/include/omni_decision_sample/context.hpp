// Copyright 2026 Boombroke
//
// Simplified context — referee data + position + nav status.
// No enemy tracking, no stance.  Pure navigation world model.
//
// Thread safety: assumes SingleThreadedExecutor or equivalent guarantee
// that all callbacks and the tick timer run on the same thread.
//
#ifndef OMNI_DECISION_SAMPLE__CONTEXT_HPP_
#define OMNI_DECISION_SAMPLE__CONTEXT_HPP_

#include <optional>

#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "omni_decision_sample/types.hpp"

namespace omni_decision_sample
{

class Context
{
public:
  Context() = default;

  void set_thresholds(const Thresholds & t) { thresholds_ = t; }
  const Thresholds & thresholds() const { return thresholds_; }

  // ========================================================================
  //  referee data updates (called from ROS subscriptions)
  // ========================================================================

  void update(const rm_interfaces::msg::GameStatus & m)
  {
    game_status_ = m;
    game_status_stamp_ = now();
  }
  void update(const rm_interfaces::msg::RobotStatus & m)
  {
    robot_status_ = m;
    robot_status_stamp_ = now();
  }
  void update(const rm_interfaces::msg::RfidStatus & m)
  {
    rfid_status_ = m;
    rfid_stamp_ = now();
  }
  void update(const rm_interfaces::msg::GameRobotHP & m)
  {
    robot_hp_ = m;
    robot_hp_stamp_ = now();
  }

  // ========================================================================
  //  referee freshness
  // ========================================================================

  bool has_referee() const { return game_status_ && robot_status_; }

  bool referee_fresh() const
  {
    if (!game_status_ || !robot_status_) return false;
    const double t = now();
    return (t - game_status_stamp_) < thresholds_.referee_stale_timeout_s &&
           (t - robot_status_stamp_) < thresholds_.referee_stale_timeout_s;
  }

  bool game_running() const
  {
    if (!game_status_) return false;
    const auto & g = *game_status_;
    return g.game_progress == rm_interfaces::msg::GameStatus::RUNNING &&
           g.stage_remain_time >= 0 &&
           g.stage_remain_time <= thresholds_.game_total_time;
  }

  int32_t remain_time() const { return game_status_ ? game_status_->stage_remain_time : -1; }

  // ========================================================================
  //  hp / ammo
  // ========================================================================

  uint16_t hp() const { return robot_status_ ? robot_status_->current_hp : 0; }
  // NOTE: serial 0x0201 does NOT report maximum_hp (always 0), so we use the
  // configured max_hp (auto sentry = 400) instead of robot_status_->maximum_hp.
  uint16_t max_hp() const { return thresholds_.max_hp; }
  bool hp_low() const { return robot_status_ && robot_status_->current_hp < thresholds_.hp_low; }
  bool hp_full() const { return robot_status_ && robot_status_->current_hp >= thresholds_.max_hp; }

  uint16_t ammo() const { return robot_status_ ? robot_status_->projectile_allowance_17mm : 0; }
  bool ammo_low() const { return robot_status_ && robot_status_->projectile_allowance_17mm <= thresholds_.ammo_low; }
  bool ammo_ok() const { return robot_status_ && robot_status_->projectile_allowance_17mm >= thresholds_.ammo_ok; }

  /// Enter RESUPPLY: low hp or low ammo.
  bool needs_resupply() const { return hp_low() || ammo_low(); }

  /// Leave RESUPPLY: fully healed AND ammo replenished past ammo_ok.
  bool resupply_done() const { return hp_full() && ammo_ok(); }

  // ========================================================================
  //  RFID / field zones
  // ========================================================================

  bool rfid_fresh() const
  {
    return rfid_status_ && (now() - rfid_stamp_) < thresholds_.referee_stale_timeout_s;
  }

  bool on_supply_pad() const
  {
    return rfid_fresh() &&
           (rfid_status_->friendly_supply_zone_non_exchange ||
            rfid_status_->friendly_supply_zone_exchange);
  }

  // ========================================================================
  //  outpost / base (for tactical route selection)
  // ========================================================================

  bool hp_data_fresh() const
  {
    return robot_hp_ && (now() - robot_hp_stamp_) < thresholds_.referee_stale_timeout_s;
  }

  bool outpost_alive() const { return hp_data_fresh() && robot_hp_->ally_outpost_hp > 0; }
  uint16_t ally_base_hp() const { return hp_data_fresh() ? robot_hp_->ally_base_hp : 0; }

  // ========================================================================
  //  navigation status
  // ========================================================================

  void set_nav_status(NavStatus s) { nav_status_ = s; }
  NavStatus nav_status() const { return nav_status_; }
  bool nav_failed() const { return nav_status_ == NavStatus::FAILED; }
  bool goal_reached() const { return nav_status_ == NavStatus::ARRIVED; }

  // ========================================================================
  //  self position (from odometry)
  // ========================================================================

  void set_sentry_position(double x, double y) { sentry_x_ = x; sentry_y_ = y; sentry_pos_valid_ = true; }
  double sentry_x() const { return sentry_x_; }
  double sentry_y() const { return sentry_y_; }
  bool sentry_pos_valid() const { return sentry_pos_valid_; }
  /// Simulate localization / TF loss (position becomes unusable). Coordinates
  /// are retained; only the valid flag drops. Used by bump-traverse tests.
  void invalidate_sentry_position() { sentry_pos_valid_ = false; }

  // ========================================================================
  //  gold (reserved for future remote-exchange decisions)
  // ========================================================================

  uint16_t gold() const { return robot_status_ ? robot_status_->remaining_gold_coin : 0; }

  // ========================================================================
  //  testing support
  // ========================================================================

  void set_now(double t) { simulated_now_ = t; }

private:
  double now() const { return simulated_now_; }

  double simulated_now_{0.0};

  std::optional<rm_interfaces::msg::GameStatus>  game_status_;
  std::optional<rm_interfaces::msg::RobotStatus>  robot_status_;
  std::optional<rm_interfaces::msg::RfidStatus>   rfid_status_;
  std::optional<rm_interfaces::msg::GameRobotHP>  robot_hp_;

  double game_status_stamp_{0.0};
  double robot_status_stamp_{0.0};
  double rfid_stamp_{0.0};
  double robot_hp_stamp_{0.0};

  NavStatus nav_status_{NavStatus::IDLE};

  double sentry_x_{0.0};
  double sentry_y_{0.0};
  bool sentry_pos_valid_{false};

  Thresholds thresholds_;
};

}  // namespace omni_decision_sample

#endif  // OMNI_DECISION_SAMPLE__CONTEXT_HPP_
