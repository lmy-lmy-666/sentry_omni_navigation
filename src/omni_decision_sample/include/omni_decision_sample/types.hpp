// Copyright 2026 Boombroke
//
// Simplified sentry decision types — navigation-only, no combat.
//
#ifndef OMNI_DECISION_SAMPLE__TYPES_HPP_
#define OMNI_DECISION_SAMPLE__TYPES_HPP_

#include <cstdint>
#include <vector>

namespace omni_decision_sample
{

/// A navigation waypoint in map frame. Orientation is omitted:
/// the gimbal auto-aims independently, chassis heading is free.
struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double dwell_s{0.0};  ///< seconds to wait after arrival
};

/// Which end of a bump (undulating/washboard) segment we are heading TO.
/// The chassis crosses the segment as a pure straight-line X translation; the
/// direction is chosen at runtime from which side the robot is currently on.
enum class BumpDir {
  FORWARD,   ///< entry.x → exit.x  (vx sign = sign(exit.x - entry.x))
  BACKWARD,  ///< exit.x → entry.x  (reverse; used on retreat)
};

using Route = std::vector<Waypoint>;

/// Top-level states — keep it minimal.
/// Supply pad doubles as the safe fallback: low hp OR low ammo both send the
/// sentry home, where it heals passively and refills ammo (+100/min, no active
/// exchange). No separate RETREAT — at supply-only granularity a "critical hp"
/// state produces the same navigation action (drive to the pad).
enum class State {
  IDLE,            ///< referee offline or game not running
  OPENING_STRIKE,  ///< match start: drive to a firing spot, dwell to let auto-aim kill enemy outpost (once)
  PATROL,          ///< default: follow patrol route
  RESUPPLY,        ///< low hp or low ammo → go to supply pad, recover, then leave
  BUMP_TRAVERSE,   ///< crossing an undulating (washboard) segment: open-loop straight dash, bypassing Nav2
};

inline const char * to_string(State s)
{
  switch (s) {
    case State::IDLE:            return "IDLE";
    case State::OPENING_STRIKE:  return "OPENING_STRIKE";
    case State::PATROL:          return "PATROL";
    case State::RESUPPLY:        return "RESUPPLY";
    case State::BUMP_TRAVERSE:   return "BUMP_TRAVERSE";
  }
  return "UNKNOWN";
}

/// Sub-phases within BUMP_TRAVERSE. The chassis is a swerve (舵轮) drive: to
/// cross undulating terrain all four wheels must point along the travel
/// direction, so the whole traversal is pure linear.x — no y, no yaw, no spin.
/// Constant speed throughout (NO end-of-segment slowdown): on washboard terrain
/// slowing down loses the momentum needed to climb the next crest, so we hold
/// dash_speed until the exit-x is crossed, then hard-stop.
enum class BumpPhase {
  GOTO_ENTRY,  ///< Nav2-drive to the entry pose (yaw pre-aligned to dash dir), wait for arrival
  ALIGN,       ///< cancel Nav2, wait align_time (swerve wheels rotate to travel dir + link goes silent)
  DASHING,     ///< open-loop constant-speed straight dash across the segment
  DONE,        ///< crossed: emit a few zero-vel frames, hand control back to the return state
  FAILED,      ///< timed out / stuck: reverse-crawl back to entry, disable this segment for the match
};

enum class NavStatus {
  IDLE,
  MOVING,
  ARRIVED,
  FAILED,
};

/// All tunable thresholds. YAML profiles can override the defaults.
struct Thresholds
{
  // --- hp / ammo (hysteresis) ------------------------------------
  // Enter RESUPPLY when hp < hp_low OR ammo <= ammo_low.
  // Leave RESUPPLY only when hp >= max_hp AND ammo >= ammo_ok.
  uint16_t max_hp{400};            ///< full-hp target (auto sentry = 400; serial 0x0201 does NOT report maximum_hp, so configure it here)
  uint16_t hp_low{150};            ///< enter RESUPPLY below this hp
  uint16_t ammo_low{50};           ///< enter RESUPPLY at/below this ammo
  uint16_t ammo_ok{100};           ///< leave RESUPPLY at/above this ammo (one free +100 cycle)

  // --- timing ----------------------------------------------------
  int32_t game_total_time{420};    ///< match duration (s)
  uint8_t min_ticks_in_state{4};   ///< min ticks before state exit (oscillation guard)

  double stuck_timeout_s{10.0};    ///< per-waypoint patrol timeout

  double resupply_timeout_s{30.0}; ///< single supply-point timeout → rotate to next point (never gives up)
  double supply_arrival_radius{0.4}; ///< m  "at the supply pad" position radius. RESUPPLY uses REAL-TIME
                                     ///< distance (not a one-shot arrival latch) so a sentry pushed out of
                                     ///< the pad — or killed & respawned away from it — re-navigates home.

  double referee_stale_timeout_s{3.0};    ///< referee data expiry

  // --- opening strike (kill enemy outpost at match start, once) ---
  double opening_strike_duration_s{90.0}; ///< dwell at firing spot to let auto-aim destroy enemy outpost (1.5 min)

  // --- bump traverse (crossing undulating / washboard terrain, open-loop) ---
  // Swerve chassis crosses as a pure straight X dash at CONSTANT speed (no
  // slowdown — washboard needs momentum). Arrival is judged on X only.
  double bump_dash_speed{0.8};     ///< m/s constant dash speed (start low, raise after real-terrain tests)
  double bump_reverse_speed{0.4};  ///< m/s reverse-crawl speed on FAILED recovery
  double bump_tol{0.25};           ///< m  X arrival tolerance (widened vs Nav2: bumpy localization jitter)
  double bump_entry_radius{0.5};   ///< m  how close to entry (XY) before we take over and cross
  double bump_y_tol{0.10};         ///< m  entry/exit y mismatch limit (segment must be axis-aligned in X)
  double bump_align_time_s{0.5};   ///< s  after Nav2 cancel: let swerve wheels align + link go silent
  double bump_timeout_s{20.0};     ///< s  dash timeout → FAILED (reverse recovery)
  int    bump_stop_ticks{3};       ///< number of zero-vel frames to emit on DONE before handing back
};

}  // namespace omni_decision_sample

#endif  // OMNI_DECISION_SAMPLE__TYPES_HPP_
