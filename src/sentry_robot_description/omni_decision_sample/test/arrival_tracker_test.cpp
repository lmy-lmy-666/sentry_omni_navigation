// Copyright 2026 Boombroke
//
// Unit tests for ArrivalTracker — the ROS-free arrival-state resolver extracted
// from DecisionNode so the fallback/Nav2 arrival race becomes testable.
//
// The headline test is Issue5_LateFeedbackDoesNotDemoteArrival: it reproduces
// the exact bug that the FSM-only suite could never catch (FSM tests set
// NavStatus directly, bypassing the node's two competing arrival paths).
//
#include "omni_decision_sample/arrival_tracker.hpp"

#include <gtest/gtest.h>

using omni_decision_sample::ArrivalTracker;
using omni_decision_sample::NavStatus;

namespace
{
constexpr double TOL = 0.25;  // matches goal_reached_distance_tolerance default
}

// =============================================================================
//  Issue #5 regression — the whole reason this class exists
// =============================================================================

TEST(ArrivalTracker, Issue5_LateFeedbackDoesNotDemoteArrival)
{
  // Nav2 feedback stream: approaching, then arrives, then a LATE frame reports a
  // larger distance_remaining (end-of-path re-plan / obstacle avoidance). That
  // late frame must NOT demote ARRIVED back to MOVING.
  ArrivalTracker t{TOL};
  t.reset();

  t.update_feedback(1.00);
  EXPECT_EQ(t.status(), NavStatus::MOVING);

  t.update_feedback(0.20);                    // within tolerance → arrived
  EXPECT_EQ(t.status(), NavStatus::ARRIVED);
  EXPECT_TRUE(t.latched());

  t.update_feedback(0.90);                    // late, larger distance
  EXPECT_EQ(t.status(), NavStatus::ARRIVED)   // must stay put
      << "late feedback with larger distance must not demote a latched arrival";
}

TEST(ArrivalTracker, Issue5_LateFeedbackAfterResultSucceeded)
{
  // A straggler feedback delivered AFTER the SUCCEEDED result must also be
  // ignored (result latches too).
  ArrivalTracker t{TOL};
  t.reset();
  t.update_feedback(0.50);
  t.on_result_succeeded();
  EXPECT_EQ(t.status(), NavStatus::ARRIVED);
  t.update_feedback(2.00);
  EXPECT_EQ(t.status(), NavStatus::ARRIVED);
}

TEST(ArrivalTracker, Issue5_FallbackArrivalNotDemotedByFeedback)
{
  // Fallback-distance path confirms arrival first; a later Nav2 feedback frame
  // (both paths coexist during the transition) must not demote it.
  ArrivalTracker t{TOL};
  t.reset();
  t.update_fallback_distance(0.10);           // odom says arrived
  EXPECT_EQ(t.status(), NavStatus::ARRIVED);
  t.update_feedback(1.50);                     // Nav2 still thinks we're moving
  EXPECT_EQ(t.status(), NavStatus::ARRIVED);
}

// =============================================================================
//  Basic behaviour
// =============================================================================

TEST(ArrivalTracker, StartsIdle)
{
  ArrivalTracker t{TOL};
  EXPECT_EQ(t.status(), NavStatus::IDLE);
  EXPECT_FALSE(t.latched());
}

TEST(ArrivalTracker, ResetGoesMoving)
{
  ArrivalTracker t{TOL};
  t.reset();
  EXPECT_EQ(t.status(), NavStatus::MOVING);
  EXPECT_FALSE(t.latched());
}

TEST(ArrivalTracker, FeedbackReportsMovingUntilWithinTolerance)
{
  ArrivalTracker t{TOL};
  t.reset();
  t.update_feedback(0.30);
  EXPECT_EQ(t.status(), NavStatus::MOVING);
  t.update_feedback(0.25);                     // == tolerance → arrived
  EXPECT_EQ(t.status(), NavStatus::ARRIVED);
}

TEST(ArrivalTracker, FallbackWithinToleranceArrives)
{
  ArrivalTracker t{TOL};
  t.reset();
  t.update_fallback_distance(0.30);
  EXPECT_FALSE(t.arrived());
  t.update_fallback_distance(0.20);
  EXPECT_TRUE(t.arrived());
}

// =============================================================================
//  New goal clears the latch (per-goal semantics)
// =============================================================================

TEST(ArrivalTracker, ResetClearsLatchForNextGoal)
{
  ArrivalTracker t{TOL};
  t.reset();
  t.update_feedback(0.10);                     // arrive at goal A
  ASSERT_TRUE(t.latched());

  t.reset();                                   // issue goal B
  EXPECT_FALSE(t.latched());
  EXPECT_EQ(t.status(), NavStatus::MOVING);
  t.update_feedback(1.00);                     // now free to report MOVING again
  EXPECT_EQ(t.status(), NavStatus::MOVING);
}

// =============================================================================
//  Result codes
// =============================================================================

TEST(ArrivalTracker, ResultFailedReportsFailedAndUnlatches)
{
  ArrivalTracker t{TOL};
  t.reset();
  t.on_result_failed();
  EXPECT_EQ(t.status(), NavStatus::FAILED);
  EXPECT_FALSE(t.latched());
}

TEST(ArrivalTracker, CanceledReportsIdleAndUnlatches)
{
  ArrivalTracker t{TOL};
  t.reset();
  t.update_feedback(0.10);                     // arrived + latched
  t.on_canceled();
  EXPECT_EQ(t.status(), NavStatus::IDLE);
  EXPECT_FALSE(t.latched());
}

TEST(ArrivalTracker, SetToleranceAffectsArrival)
{
  ArrivalTracker t{0.25};
  t.reset();
  t.update_feedback(0.40);
  EXPECT_EQ(t.status(), NavStatus::MOVING);
  t.set_tolerance(0.50);                       // widen tolerance
  t.update_feedback(0.40);                     // now within → arrived
  EXPECT_EQ(t.status(), NavStatus::ARRIVED);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
