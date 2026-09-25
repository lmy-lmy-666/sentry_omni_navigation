// Copyright 2026 Boombroke
//
// YAML profile loader for simplified sentry decision.
//
#ifndef OMNI_DECISION_SAMPLE__PROFILE_HPP_
#define OMNI_DECISION_SAMPLE__PROFILE_HPP_

#include <string>
#include <vector>

#include "omni_decision_sample/types.hpp"

namespace omni_decision_sample
{

/// An undulating (washboard) terrain segment the sentry must cross open-loop.
/// The two poses are the flat ground just BEFORE and AFTER the bumps. The
/// segment must be axis-aligned in X: |entry.y - exit.y| <= bump_y_tol.
/// yaw is the chassis heading to face while dashing (radians, map frame).
struct BumpSegment
{
  Waypoint entry;      ///< flat ground on one side (Nav2 drives here first, pre-aligned to yaw)
  Waypoint exit;       ///< flat ground on the other side (dash target, judged on X)
  double   yaw{0.0};   ///< heading to hold while crossing (align swerve wheels to travel dir)
};

struct Profile
{
  std::string name;
  Thresholds thresholds;

  // --- mandatory waypoints ---------------------------------------
  Route  patrol;             ///< default patrol loop
  Waypoint supply;           ///< supply pad — the sole recovery destination (heal + refill)

  // --- opening strike (optional) ---------------------------------
  Waypoint opening_strike;   ///< match-start firing spot to hit enemy outpost
  bool     has_opening_strike{false};  ///< true if 'opening_strike' is configured

  // --- optional tactical variant (selected at runtime) ----------
  Route patrol_aggressive;   ///< used while our outpost is alive (前哨存活激进前压)
                             ///< when outpost is destroyed → fall back to `patrol` (我方半场防守)

  // --- fallback chains -------------------------------------------
  std::vector<Waypoint> backup_supply_points;

  // --- undulating terrain segments (crossed open-loop, bypassing Nav2) ------
  std::vector<BumpSegment> bump_segments;
};

/// Parse a YAML profile. Throws std::runtime_error on invalid input.
Profile load_profile(const std::string & yaml_path);

}  // namespace omni_decision_sample

#endif  // OMNI_DECISION_SAMPLE__PROFILE_HPP_
