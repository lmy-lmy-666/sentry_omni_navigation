// Copyright 2026 Boombroke
//
// ArrivalTracker — pure (no-ROS) arrival-state resolver for a single nav goal.
//
// WHY THIS EXISTS
// ---------------
// Goal arrival is fed by two asynchronous paths that both wrote the same
// nav_status field in DecisionNode:
//   1. fallback odom-distance check (when Nav2 action is unavailable)
//   2. Nav2 action feedback (distance_remaining) + result
// After arrival was detected, a *later* Nav2 feedback frame with a larger
// distance_remaining (e.g. end-of-path re-planning / obstacle avoidance) could
// demote ARRIVED back to MOVING — a race the FSM unit tests never covered
// because they set NavStatus directly, bypassing the node entirely.
//
// This class makes arrival LATCHED per goal: once any path confirms ARRIVED,
// no later feedback may demote it. Being ROS-free, the race is now unit-
// testable (see arrival_tracker_test.cpp).
//
#ifndef OMNI_DECISION_SAMPLE__ARRIVAL_TRACKER_HPP_
#define OMNI_DECISION_SAMPLE__ARRIVAL_TRACKER_HPP_

#include "omni_decision_sample/types.hpp"

namespace omni_decision_sample
{

class ArrivalTracker
{
public:
  explicit ArrivalTracker(double tolerance = 0.25) : tolerance_(tolerance) {}

  /// Arrival distance tolerance (m). Set from the ROS parameter after the node
  /// has read it (the tracker is a default-constructed member).
  void set_tolerance(double tolerance) { tolerance_ = tolerance; }

  /// Call when a NEW goal is issued: clears the latch, marks us moving.
  void reset()
  {
    latched_ = false;
    status_ = NavStatus::MOVING;
  }

  /// Fallback odom-distance arrival check (Nav2 action unavailable path).
  /// Latches ARRIVED when within tolerance. No-op once already latched.
  void update_fallback_distance(double distance)
  {
    if (latched_) return;
    if (distance <= tolerance_) {
      status_ = NavStatus::ARRIVED;
      latched_ = true;
    }
  }

  /// Nav2 action feedback (distance_remaining). Latches ARRIVED when within
  /// tolerance; otherwise reports MOVING — but NEVER demotes a latched arrival
  /// back to MOVING (this is the race fix). No-op once latched.
  void update_feedback(double distance_remaining)
  {
    if (latched_) return;   // already arrived — ignore late feedback
    if (distance_remaining <= tolerance_) {
      status_ = NavStatus::ARRIVED;
      latched_ = true;
    } else {
      status_ = NavStatus::MOVING;
    }
  }

  /// Nav2 action result: SUCCEEDED. Latches ARRIVED (guards against any
  /// straggler feedback delivered after the result).
  void on_result_succeeded()
  {
    status_ = NavStatus::ARRIVED;
    latched_ = true;
  }

  /// Nav2 action result: ABORTED / UNKNOWN (goal failed).
  void on_result_failed()
  {
    status_ = NavStatus::FAILED;
    latched_ = false;
  }

  /// Nav2 action result: CANCELED, or an explicit cancel_nav().
  void on_canceled()
  {
    status_ = NavStatus::IDLE;
    latched_ = false;
  }

  NavStatus status() const { return status_; }
  bool arrived() const { return status_ == NavStatus::ARRIVED; }
  bool latched() const { return latched_; }

private:
  double    tolerance_;
  bool      latched_{false};
  NavStatus status_{NavStatus::IDLE};
};

}  // namespace omni_decision_sample

#endif  // OMNI_DECISION_SAMPLE__ARRIVAL_TRACKER_HPP_
