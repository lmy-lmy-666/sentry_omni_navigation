// Copyright 2026 Boombroke
//
// Simplified FSM — 3 states, no combat, no stance.
// IDLE → RESUPPLY → PATROL (priority order).
//
#ifndef OMNI_DECISION_SAMPLE__FSM_HPP_
#define OMNI_DECISION_SAMPLE__FSM_HPP_

#include <cstddef>
#include <functional>
#include <vector>

#include "omni_decision_sample/context.hpp"
#include "omni_decision_sample/profile.hpp"
#include "omni_decision_sample/types.hpp"

namespace omni_decision_sample
{

using GoalPublisher = std::function<void(const Waypoint &)>;
using NavCanceller  = std::function<void()>;
/// Publish an open-loop chassis velocity (linear.x only) while crossing a bump
/// segment. vx is signed (m/s); y and yaw are always zero (swerve constraint).
using BumpVelPublisher = std::function<void(double vx)>;

class DecisionFsm
{
public:
  DecisionFsm(
    Profile profile, GoalPublisher goal_pub, NavCanceller nav_cancel = nullptr,
    BumpVelPublisher bump_vel_pub = nullptr);

  /// Main entry point — call at tick_frequency (default 10 Hz).
  void tick(const Context & ctx, double now_s);

  State state() const { return state_; }
  const char * state_reason() const { return state_reason_; }
  const Profile & profile() const { return profile_; }

private:
  // --- state machine -----------------------------------------------
  State select_state(const Context & ctx, double now_s);
  bool  can_leave_current_state(State next) const;

  void on_enter(State s, double now_s);
  void on_exit(State s);
  void run_behaviour(const Context & ctx, double now_s);

  // --- behaviours --------------------------------------------------
  void behave_idle(const Context & ctx, double now_s);
  void behave_opening_strike(const Context & ctx, double now_s);
  void behave_patrol(const Context & ctx, double now_s);
  void behave_resupply(const Context & ctx, double now_s);
  void behave_bump_traverse(const Context & ctx, double now_s);

  // --- bump traverse helpers ---------------------------------------
  /// Scan bump_segments; if the robot is near a segment's near-side entry AND
  /// its current business goal lies across the segment, pick it and set the
  /// dash direction. Returns the segment index, or -1 if none applies.
  int  find_bump_to_cross(const Context & ctx, double goal_x) const;
  const Waypoint & bump_dash_target() const;   ///< the exit we are dashing toward (dir-aware)
  const Waypoint & bump_entry_target() const;  ///< the entry pose (Nav2 pre-align target)
  double bump_dash_vx() const;                 ///< signed constant dash speed (dir-aware)
  bool   bump_reached(const Context & ctx) const;  ///< X crossed the dash target (single-sided)

  // --- navigation helpers ------------------------------------------
  void drive_route(const Route & route, double now_s);
  void publish_single_goal(const Waypoint & wp, double now_s);

  // --- supply target rotation (primary pad + backups, cycling) -----
  const Waypoint & current_supply_target() const;
  void advance_supply_target(double now_s);

  // ==================================================================
  //  members
  // ==================================================================

  Profile          profile_;
  GoalPublisher    publish_goal_;
  NavCanceller     cancel_nav_;
  BumpVelPublisher publish_bump_vel_;

  State state_{State::IDLE};

  // route tracking
  std::size_t   path_idx_{0};
  bool          goal_sent_{false};
  const Route * active_route_{nullptr};  ///< route in use last tick (detect tactical switch)
  double      waypoint_started_s_{0.0};
  double      waypoint_arrived_s_{0.0};
  bool        goal_arrived_{false};

  // state timing
  int    ticks_in_state_{0};

  // OPENING_STRIKE state (match-start outpost strike, done once per match)
  bool   opening_done_{false};      ///< true once the opening strike has been consumed
  double opening_entered_s_{0.0};   ///< when OPENING_STRIKE was entered (for dwell timing)

  // RESUPPLY state
  uint8_t     rfid_window_{0};        ///< RFID debounce: 5-tick sliding window, ≥3 hits → confirmed
  std::size_t supply_backup_idx_{0};  ///< current backup supply point index (own variable, not path_idx_)
  double      operation_started_s_{0.0};  ///< current supply-point attempt start time

  // BUMP_TRAVERSE state
  BumpPhase bump_phase_{BumpPhase::GOTO_ENTRY};
  int       bump_seg_idx_{-1};        ///< index into profile_.bump_segments being crossed
  BumpDir   bump_dir_{BumpDir::FORWARD};
  double    bump_phase_started_s_{0.0};  ///< entry time of ALIGN / DASHING (for align + timeout)
  double    bump_last_vx_{0.0};       ///< last commanded vx (held during TF/position dropouts)
  int       bump_stop_frames_{0};     ///< zero-vel frames emitted so far in DONE
  std::vector<bool> bump_disabled_;   ///< per-segment: crossing failed this match → don't retry

  // logging
  const char * state_reason_{""};     ///< why was the current state selected
};

}  // namespace omni_decision_sample

#endif  // OMNI_DECISION_SAMPLE__FSM_HPP_
