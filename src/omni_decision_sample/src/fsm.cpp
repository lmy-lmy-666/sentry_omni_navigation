// Copyright 2026 Boombroke
//
// Simplified FSM — 3 states, navigation-only.
//
// Priority chain (evaluated every tick, first match wins):
//   ① IDLE           — referee offline or game not running
//   ② RESUPPLY       — hp < hp_low (150) OR ammo ≤ ammo_low (50).
//                      Drive to supply pad, recover passively (heal + free
//                      +100/min ammo). Exit only when hp == max AND ammo ≥
//                      ammo_ok (100). Never gives up (covers respawn).
//   ③ OPENING_STRIKE — match start, once: drive to a firing spot and dwell so
//                      auto-aim can destroy the enemy outpost. Preempted by
//                      RESUPPLY (low hp/ammo). Consumed after dwell or preempt.
//   ④ PATROL         — default: follow patrol route
//
#include "omni_decision_sample/fsm.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace omni_decision_sample
{

DecisionFsm::DecisionFsm(
  Profile profile, GoalPublisher goal_pub, NavCanceller nav_cancel,
  BumpVelPublisher bump_vel_pub)
: profile_(std::move(profile)),
  publish_goal_(std::move(goal_pub)),
  cancel_nav_(std::move(nav_cancel)),
  publish_bump_vel_(std::move(bump_vel_pub))
{
  bump_disabled_.assign(profile_.bump_segments.size(), false);
}

// ============================================================================
//  tick
// ============================================================================

void DecisionFsm::tick(const Context & ctx, double now_s)
{
  const State candidate = select_state(ctx, now_s);
  const State next = can_leave_current_state(candidate) ? candidate : state_;

  if (next != state_) {
    on_exit(state_);
    state_ = next;
    on_enter(next, now_s);
  }

  ++ticks_in_state_;
  run_behaviour(ctx, now_s);
}

// ============================================================================
//  select_state — priority chain
// ============================================================================

State DecisionFsm::select_state(const Context & ctx, double now_s)
{
  (void)now_s;

  // ① IDLE — referee down or game stopped. Highest priority: it is the ONLY
  //    thing allowed to preempt an in-progress bump dash (emergency stop).
  if (!ctx.referee_fresh() || !ctx.game_running()) {
    state_reason_ = "referee stale / game not running";
    return State::IDLE;
  }

  // ② BUMP_TRAVERSE — once crossing, stay until DONE/FAILED. Never leave
  //    mid-dash for hp/ammo/anything (leaving = stranded on the bumps). Only
  //    IDLE above may preempt.
  if (state_ == State::BUMP_TRAVERSE) {
    const bool crossing = bump_phase_ != BumpPhase::DONE && bump_phase_ != BumpPhase::FAILED;
    // DONE: stay a few more ticks to emit bump_stop_ticks zero-vel frames so the
    // chassis settles before handing control back (FAILED hands back immediately).
    const bool settling =
      bump_phase_ == BumpPhase::DONE && bump_stop_frames_ < profile_.thresholds.bump_stop_ticks;
    if (crossing || settling) {
      state_reason_ = crossing ? "crossing undulating segment" : "settling after crossing";
      return State::BUMP_TRAVERSE;
    }
    // DONE (settled) / FAILED → fall through to re-evaluate the business state.
  }

  // Determine the business state + its destination x, then check whether a bump
  // segment sits between us and that destination (→ BUMP_TRAVERSE).

  // ③ RESUPPLY — stay until fully recovered (hysteresis).
  //    Exit only when hp == max AND ammo ≥ ammo_ok. This also covers respawn:
  //    a dead sentry reads hp 0 → stays in RESUPPLY → keeps heading home after
  //    it revives, until fully healed and rearmed.
  State   business = State::PATROL;
  double  goal_x   = 0.0;
  const char * reason = "default";

  if (state_ == State::RESUPPLY && !ctx.resupply_done()) {
    business = State::RESUPPLY;
    goal_x   = current_supply_target().x;
    reason   = "recovering (hp/ammo not yet full)";
  } else if (state_ != State::RESUPPLY && ctx.needs_resupply()) {
    // Enter RESUPPLY when hp/ammo low. Also preempts (consumes) OPENING_STRIKE.
    if (state_ == State::OPENING_STRIKE) opening_done_ = true;
    business = State::RESUPPLY;
    goal_x   = current_supply_target().x;
    reason   = ctx.hp_low() ? "hp low" : "ammo low";
  } else if (state_ == State::OPENING_STRIKE && !opening_done_) {
    business = State::OPENING_STRIKE;
    goal_x   = profile_.opening_strike.x;
    reason   = "opening strike (killing enemy outpost)";
  } else if (state_ != State::OPENING_STRIKE && !opening_done_ && profile_.has_opening_strike) {
    business = State::OPENING_STRIKE;
    goal_x   = profile_.opening_strike.x;
    reason   = "opening strike start";
  } else {
    business = State::PATROL;
    // representative destination: first waypoint of the active patrol route
    const Route & r = (ctx.outpost_alive() && !profile_.patrol_aggressive.empty())
                        ? profile_.patrol_aggressive : profile_.patrol;
    goal_x = r.empty() ? ctx.sentry_x() : r.front().x;
    reason = "default";
  }

  // ④ BUMP_TRAVERSE — if an undulating segment separates us from goal_x, cross
  //    it first. OPENING_STRIKE is never routed through a bump (opening spot is
  //    in our own half); only PATROL (前压/撤退) and RESUPPLY may need to cross.
  if (business != State::OPENING_STRIKE) {
    const int seg = find_bump_to_cross(ctx, goal_x);
    if (seg >= 0) {
      bump_seg_idx_ = seg;
      // Dash direction from which side we are on: FORWARD = toward exit.
      const BumpSegment & bs = profile_.bump_segments[seg];
      const double lo = std::min(bs.entry.x, bs.exit.x);
      const bool robot_low = ctx.sentry_x() <= lo;
      const bool exit_is_high = bs.exit.x >= bs.entry.x;
      bump_dir_ = (robot_low == exit_is_high) ? BumpDir::FORWARD : BumpDir::BACKWARD;
      state_reason_ = "bump segment between us and goal";
      return State::BUMP_TRAVERSE;
    }
  }

  state_reason_ = reason;
  return business;
}

// ============================================================================
//  find_bump_to_cross — is a bump segment between the robot and goal_x?
// ============================================================================

int DecisionFsm::find_bump_to_cross(const Context & ctx, double goal_x) const
{
  if (!ctx.sentry_pos_valid() || profile_.bump_segments.empty()) return -1;

  const double rx = ctx.sentry_x();
  const double ry = ctx.sentry_y();

  for (std::size_t i = 0; i < profile_.bump_segments.size(); ++i) {
    if (bump_disabled_[i]) continue;
    const BumpSegment & s = profile_.bump_segments[i];

    const double lo = std::min(s.entry.x, s.exit.x);
    const double hi = std::max(s.entry.x, s.exit.x);

    // y-corridor gate: only consider a segment we are roughly lined up with,
    // so a bump elsewhere on the map can't trigger. entry.y ≈ exit.y (validated).
    if (std::abs(ry - s.entry.y) > profile_.thresholds.bump_entry_radius) continue;

    // robot and goal must be on OPPOSITE sides of the segment's x-span.
    const int robot_side = rx < lo ? -1 : (rx > hi ? +1 : 0);
    const int goal_side  = goal_x < lo ? -1 : (goal_x > hi ? +1 : 0);
    if (robot_side == 0 || goal_side == 0 || robot_side == goal_side) continue;

    return static_cast<int>(i);
  }
  return -1;
}

// ============================================================================
//  can_leave_current_state — oscillation guard
// ============================================================================

bool DecisionFsm::can_leave_current_state(State next) const
{
  if (next == state_) return true;

  // IDLE (game not running) always preempts, and is always leavable. This is
  // the ONLY thing allowed to interrupt a bump crossing (emergency stop).
  if (next == State::IDLE) return true;
  if (state_ == State::IDLE) return true;

  // BUMP_TRAVERSE is preempt-proof: while crossing (not yet DONE/FAILED), no
  // state but IDLE (handled above) may take over — leaving = stranded on the
  // bumps. Once DONE/FAILED, allow the handover to the business state.
  if (state_ == State::BUMP_TRAVERSE) {
    return bump_phase_ == BumpPhase::DONE || bump_phase_ == BumpPhase::FAILED;
  }

  // RESUPPLY (low hp / low ammo) preempts PATROL immediately.
  if (next == State::RESUPPLY) return true;

  // Otherwise require minimum ticks to dampen oscillation
  return ticks_in_state_ >= profile_.thresholds.min_ticks_in_state;
}

// ============================================================================
//  on_enter / on_exit
// ============================================================================

void DecisionFsm::on_enter(State s, double now_s)
{
  path_idx_ = 0;
  goal_sent_ = false;
  goal_arrived_ = false;
  waypoint_started_s_ = now_s;
  waypoint_arrived_s_ = 0.0;
  ticks_in_state_ = 0;

  switch (s) {
    case State::IDLE:
      if (cancel_nav_) cancel_nav_();
      break;
    case State::OPENING_STRIKE:
      opening_entered_s_ = now_s;
      break;
    case State::RESUPPLY:
      operation_started_s_ = now_s;
      rfid_window_ = 0;
      supply_backup_idx_ = 0;
      break;
    case State::PATROL:
      break;
    case State::BUMP_TRAVERSE:
      // bump_dir_ / bump_seg_idx_ were set in select_state (has ctx). Here we
      // just reset the phase machine.
      bump_phase_ = BumpPhase::GOTO_ENTRY;
      bump_phase_started_s_ = now_s;
      bump_last_vx_ = 0.0;
      bump_stop_frames_ = 0;
      break;
  }
}

void DecisionFsm::on_exit(State)
{
}

// ============================================================================
//  run_behaviour — dispatch
// ============================================================================

void DecisionFsm::run_behaviour(const Context & ctx, double now_s)
{
  // Detect goal arrival (Nav2 feedback or fallback odom check)
  if (goal_sent_ && !goal_arrived_ && ctx.goal_reached()) {
    goal_arrived_ = true;
    waypoint_arrived_s_ = now_s;
  }

  switch (state_) {
    case State::IDLE:            behave_idle(ctx, now_s);           break;
    case State::OPENING_STRIKE:  behave_opening_strike(ctx, now_s); break;
    case State::PATROL:          behave_patrol(ctx, now_s);         break;
    case State::RESUPPLY:        behave_resupply(ctx, now_s);       break;
    case State::BUMP_TRAVERSE:   behave_bump_traverse(ctx, now_s);  break;
  }
}

// ============================================================================
//  behave_idle
// ============================================================================

void DecisionFsm::behave_idle(const Context &, double)
{
  // Navigation was cancelled in on_enter.  Nothing to do while idle.
}

// ============================================================================
//  behave_opening_strike — drive to the firing spot, dwell to let auto-aim
//  destroy the enemy outpost, then consume this once-per-match action.
// ============================================================================

void DecisionFsm::behave_opening_strike(const Context & ctx, double now_s)
{
  // Dwell elapsed → strike consumed; select_state will fall through to PATROL.
  if (now_s - opening_entered_s_ >= profile_.thresholds.opening_strike_duration_s) {
    opening_done_ = true;
    return;
  }

  // Drive to the firing spot once; hold there while auto-aim fires.
  if (!goal_sent_) {
    publish_single_goal(profile_.opening_strike, now_s);
    return;
  }

  // Nav stuck en route → retry (re-plan). We keep trying: the strike spot is in
  // our own half and should be reachable; if truly blocked, the dwell timer
  // still expires and we move on.
  if (ctx.nav_failed()) {
    goal_sent_ = false;
    publish_single_goal(profile_.opening_strike, now_s);
  }
}

// ============================================================================
//  behave_patrol — follow patrol route, switch variants tactically
// ============================================================================

void DecisionFsm::behave_patrol(const Context & ctx, double now_s)
{
  // Tactical route selection driven by our outpost:
  //   outpost alive     → patrol_aggressive (前压), if defined
  //   outpost destroyed → patrol (我方半场防守)
  const Route * route = &profile_.patrol;
  if (ctx.outpost_alive() && !profile_.patrol_aggressive.empty()) {
    route = &profile_.patrol_aggressive;
  }

  // Route switched since last tick → reset waypoint tracking so we don't
  // judge arrival/timeout of the OLD goal against the NEW route's indices,
  // and don't chase a stale goal. Republish from the new route's start.
  if (route != active_route_) {
    active_route_ = route;
    path_idx_ = 0;
    goal_sent_ = false;
    goal_arrived_ = false;
  }

  if (route->empty()) {
    goal_sent_ = true;  // prevent busy-looping
    return;
  }

  if (ctx.nav_failed()) {
    path_idx_ = (path_idx_ + 1) % route->size();
    goal_sent_ = false;
    return;
  }

  drive_route(*route, now_s);
}

// ============================================================================
//  behave_resupply — navigate to supply pad, heal, then leave
// ============================================================================

void DecisionFsm::behave_resupply(const Context & ctx, double now_s)
{
  // --- "at the supply pad?" — REAL-TIME position, NOT a one-shot arrival latch.
  // Why not goal_arrived_: that latch means "we reached SOME goal once" and is
  // never cleared while we stay in RESUPPLY. If the sentry is then pushed out of
  // the pad, or killed and respawned AWAY from it, goal_arrived_ stays true and
  // the old code would `return` forever — healing in place off-pad, never
  // re-navigating home. Judging on live distance fixes both cases: leave the
  // pad → we're no longer "arrived" → we re-issue the goal and drive back.
  //
  // Two confirmations (either suffices):
  //   • position: within supply_arrival_radius of the current supply target
  //   • RFID: friendly-supply-zone bit, 5-tick sliding window ≥3 hits (only
  //     meaningful if the referee RFID field is wired correctly upstream)
  rfid_window_ = static_cast<uint8_t>((rfid_window_ << 1) & 0x1F);
  if (ctx.on_supply_pad()) rfid_window_ |= 1;
  const bool rfid_confirmed = __builtin_popcount(rfid_window_) >= 3;

  bool at_pad = rfid_confirmed;
  if (!at_pad && ctx.sentry_pos_valid()) {
    const Waypoint & tgt = current_supply_target();
    const double dx = ctx.sentry_x() - tgt.x;
    const double dy = ctx.sentry_y() - tgt.y;
    at_pad = std::hypot(dx, dy) <= profile_.thresholds.supply_arrival_radius;
  }

  if (at_pad) {
    // On the pad → stay put, heal + refill ammo passively. Drop goal_sent_ so
    // that if we later leave the pad (pushed off / respawn), the block below
    // immediately re-issues the navigation goal instead of waiting on a stale
    // "already sent" flag.
    goal_sent_ = false;
    return;
  }

  // --- navigation: keep heading to the supply pad, never give up ---
  // The pad is the only safe destination. We rotate through backup points on
  // stuck/timeout, then wrap back to the primary — so a respawned sentry (hp
  // recovered from 0, possibly off-pad) always resumes navigating home.

  // Nav stuck → advance to next candidate immediately.
  if (ctx.nav_failed()) {
    advance_supply_target(now_s);
    return;
  }

  // Not on the pad and no goal in flight → (re)issue the primary supply goal.
  // This is what re-triggers navigation after being pushed off / respawning.
  if (!goal_sent_) {
    publish_single_goal(current_supply_target(), now_s);
    operation_started_s_ = now_s;
    return;
  }

  // Single-point timeout → rotate to next candidate.
  if (now_s - operation_started_s_ > profile_.thresholds.resupply_timeout_s) {
    advance_supply_target(now_s);
  }
}

// ============================================================================
//  bump traverse helpers
// ============================================================================

const Waypoint & DecisionFsm::bump_entry_target() const
{
  const BumpSegment & s = profile_.bump_segments[bump_seg_idx_];
  // The entry we drive Nav2 to is the near side: for FORWARD we approach from
  // the entry end; for BACKWARD from the exit end.
  return (bump_dir_ == BumpDir::FORWARD) ? s.entry : s.exit;
}

const Waypoint & DecisionFsm::bump_dash_target() const
{
  const BumpSegment & s = profile_.bump_segments[bump_seg_idx_];
  return (bump_dir_ == BumpDir::FORWARD) ? s.exit : s.entry;
}

double DecisionFsm::bump_dash_vx() const
{
  const double target_x = bump_dash_target().x;
  const double from_x   = bump_entry_target().x;
  const double v = profile_.thresholds.bump_dash_speed;
  return (target_x >= from_x) ? v : -v;   // constant speed, signed by direction
}

bool DecisionFsm::bump_reached(const Context & ctx) const
{
  // Single-sided X comparison toward the dash target (y/yaw ignored — swerve
  // straight dash). Tolerance widened to absorb washboard localization jitter.
  const double target_x = bump_dash_target().x;
  const double tol = profile_.thresholds.bump_tol;
  return (bump_dash_vx() > 0.0) ? (ctx.sentry_x() >= target_x - tol)
                                : (ctx.sentry_x() <= target_x + tol);
}

// ============================================================================
//  behave_bump_traverse — cross undulating terrain open-loop (5 phases)
//
//  Swerve chassis: pure linear.x, constant speed (no slowdown — washboard needs
//  momentum), y/yaw = 0. Nav2 is cancelled during the dash; the chassis cmd_vel
//  topic goes silent on its own (fake_vel_transform is callback-driven), so the
//  decision node is the sole speed source while dashing.
// ============================================================================

void DecisionFsm::behave_bump_traverse(const Context & ctx, double now_s)
{
  // Loop so a phase transition takes effect THIS tick: a case that advances the
  // phase does `continue` to re-run the switch; a case that is done does `return`.
  for (;;) {
  switch (bump_phase_) {
    case BumpPhase::GOTO_ENTRY: {
      // Nav2-drive to the entry pose (Nav2 pre-aligns chassis yaw to dash dir).
      if (!goal_sent_) {
        publish_single_goal(bump_entry_target(), now_s);
        return;
      }
      // Close enough to the entry (XY) → cancel Nav2 and start aligning.
      const double dx = ctx.sentry_x() - bump_entry_target().x;
      const double dy = ctx.sentry_y() - bump_entry_target().y;
      const bool near_entry = std::hypot(dx, dy) <= profile_.thresholds.bump_entry_radius;
      if (goal_arrived_ || near_entry) {
        if (cancel_nav_) cancel_nav_();
        bump_phase_ = BumpPhase::ALIGN;
        bump_phase_started_s_ = now_s;
        return;
      }
      // Nav2 failed to reach entry → retry once, else fail out.
      if (ctx.nav_failed()) {
        goal_sent_ = false;
        publish_single_goal(bump_entry_target(), now_s);
      }
      return;
    }

    case BumpPhase::ALIGN: {
      // Once aligned, transition to DASHING and run it THIS tick (fall through
      // via the enclosing loop) so the dash velocity is issued immediately.
      if (now_s - bump_phase_started_s_ >= profile_.thresholds.bump_align_time_s) {
        bump_phase_ = BumpPhase::DASHING;
        bump_phase_started_s_ = now_s;
        bump_last_vx_ = bump_dash_vx();
        continue;
      }
      // Still aligning: hold zero velocity (swerve wheels rotating, link drains).
      if (publish_bump_vel_) publish_bump_vel_(0.0);
      return;
    }

    case BumpPhase::DASHING: {
      // Timeout guard → go to reverse recovery, run it this tick.
      if (now_s - bump_phase_started_s_ > profile_.thresholds.bump_timeout_s) {
        bump_phase_ = BumpPhase::FAILED;
        bump_phase_started_s_ = now_s;
        continue;
      }
      // Position dropout (bumpy point cloud → TF loss is normal): keep last
      // commanded velocity, do NOT stop — stopping risks stalling in a trough.
      if (!ctx.sentry_pos_valid()) {
        if (publish_bump_vel_) publish_bump_vel_(bump_last_vx_);
        return;
      }
      // Crossed the exit x → hard stop (constant speed until here, no ramp).
      if (bump_reached(ctx)) {
        if (publish_bump_vel_) publish_bump_vel_(0.0);
        bump_phase_ = BumpPhase::DONE;
        bump_stop_frames_ = 0;
        return;
      }
      // Constant-speed straight dash.
      bump_last_vx_ = bump_dash_vx();
      if (publish_bump_vel_) publish_bump_vel_(bump_last_vx_);
      return;
    }

    case BumpPhase::DONE: {
      // Emit a few zero-vel frames to settle, then select_state hands control
      // back to bump_return_state_ (can_leave_current_state now allows exit).
      if (publish_bump_vel_) publish_bump_vel_(0.0);
      ++bump_stop_frames_;
      return;
    }

    case BumpPhase::FAILED: {
      // Reverse-crawl back to the entry side at half speed, same open-loop rule.
      const double target_x = bump_entry_target().x;
      const double tol = profile_.thresholds.bump_tol;
      const double rv = profile_.thresholds.bump_reverse_speed;
      const double vx = (target_x >= bump_dash_target().x) ? rv : -rv;

      // Reverse timeout → give up: stop, disable this segment, hand back.
      if (now_s - bump_phase_started_s_ > profile_.thresholds.bump_timeout_s) {
        if (publish_bump_vel_) publish_bump_vel_(0.0);
        if (bump_seg_idx_ >= 0) bump_disabled_[bump_seg_idx_] = true;
        return;  // stays FAILED → select_state falls through to business state
      }
      if (!ctx.sentry_pos_valid()) {
        if (publish_bump_vel_) publish_bump_vel_(vx);
        return;
      }
      const bool back = (vx > 0.0) ? (ctx.sentry_x() >= target_x - tol)
                                   : (ctx.sentry_x() <= target_x + tol);
      if (back) {
        if (publish_bump_vel_) publish_bump_vel_(0.0);
        if (bump_seg_idx_ >= 0) bump_disabled_[bump_seg_idx_] = true;
        return;  // recovered to entry; segment disabled for the match
      }
      if (publish_bump_vel_) publish_bump_vel_(vx);
      return;
    }
  }
  }  // for(;;)
}

// ============================================================================
//  supply target rotation — primary pad + backups, cycling forever
// ============================================================================

const Waypoint & DecisionFsm::current_supply_target() const
{
  // index 0 = primary supply pad; 1.. = backup_supply_points
  if (supply_backup_idx_ == 0 || profile_.backup_supply_points.empty()) {
    return profile_.supply;
  }
  const std::size_t i = (supply_backup_idx_ - 1) % profile_.backup_supply_points.size();
  return profile_.backup_supply_points[i];
}

void DecisionFsm::advance_supply_target(double now_s)
{
  // Cycle: primary → backup[0] → … → backup[n-1] → primary → …
  const std::size_t total = profile_.backup_supply_points.size() + 1;  // +1 for primary
  supply_backup_idx_ = (supply_backup_idx_ + 1) % total;
  goal_sent_ = false;
  publish_single_goal(current_supply_target(), now_s);
  operation_started_s_ = now_s;
}

// ============================================================================
//  drive_route — sequential waypoint traversal with dwell
// ============================================================================

void DecisionFsm::drive_route(const Route & route, double now_s)
{
  if (route.empty()) return;
  if (path_idx_ >= route.size()) path_idx_ = 0;

  const Waypoint & wp = route[path_idx_];

  // Send goal
  if (!goal_sent_) {
    publish_goal_(wp);
    goal_sent_ = true;
    goal_arrived_ = false;
    waypoint_started_s_ = now_s;
    waypoint_arrived_s_ = 0.0;
    return;
  }

  // Per-waypoint stuck timeout → skip to next
  if (!goal_arrived_ && now_s - waypoint_started_s_ > profile_.thresholds.stuck_timeout_s) {
    path_idx_ = (path_idx_ + 1) % route.size();
    goal_sent_ = false;
    return;
  }

  // Waiting for arrival
  if (!goal_arrived_) return;

  // Dwell at waypoint
  if (now_s - waypoint_arrived_s_ >= wp.dwell_s) {
    path_idx_ = (path_idx_ + 1) % route.size();
    goal_sent_ = false;
    goal_arrived_ = false;
  }
}

// ============================================================================
//  publish_single_goal — one-shot waypoint
// ============================================================================

void DecisionFsm::publish_single_goal(const Waypoint & wp, double now_s)
{
  if (!goal_sent_) {
    publish_goal_(wp);
    goal_sent_ = true;
    goal_arrived_ = false;
    waypoint_started_s_ = now_s;
    waypoint_arrived_s_ = 0.0;
  }
}

}  // namespace omni_decision_sample
