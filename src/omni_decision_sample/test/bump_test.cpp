// Copyright 2026 Boombroke
//
// Unit tests for BUMP_TRAVERSE — crossing undulating (washboard) terrain
// open-loop, bypassing Nav2. Swerve chassis: pure linear.x, CONSTANT speed
// (no slowdown), y/yaw = 0. Position-aware auto-trigger.
//
// These tests exercise the FSM layer with injected callbacks (goal / cancel /
// bump-vel), the same pattern as fsm_test — no ROS needed.
//
#include "omni_decision_sample/fsm.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "omni_decision_sample/context.hpp"
#include "omni_decision_sample/profile.hpp"
#include "omni_decision_sample/types.hpp"

using omni_decision_sample::BumpSegment;
using omni_decision_sample::Context;
using omni_decision_sample::DecisionFsm;
using omni_decision_sample::NavStatus;
using omni_decision_sample::Profile;
using omni_decision_sample::State;
using omni_decision_sample::Waypoint;

namespace
{

constexpr uint8_t RUNNING = rm_interfaces::msg::GameStatus::RUNNING;
constexpr uint16_t FULL_HP = 400;
constexpr uint16_t FULL_AMMO = 300;

rm_interfaces::msg::GameStatus game(uint8_t progress, int32_t remain)
{
  rm_interfaces::msg::GameStatus m;
  m.game_progress = progress;
  m.stage_remain_time = remain;
  return m;
}

rm_interfaces::msg::RobotStatus robot(uint16_t hp, uint16_t ammo)
{
  rm_interfaces::msg::RobotStatus m;
  m.current_hp = hp;
  m.maximum_hp = 400;
  m.projectile_allowance_17mm = ammo;
  return m;
}

// A profile with one X-aligned bump segment from x=1 to x=5 at y=0.
// Patrol goal sits on the far side (x=8) so a robot at x=0 must cross.
Profile bump_profile()
{
  Profile p;
  p.name = "bump_test";
  p.thresholds.min_ticks_in_state = 1;   // fast tests
  p.thresholds.bump_dash_speed = 1.0;
  p.thresholds.bump_reverse_speed = 0.5;
  p.thresholds.bump_tol = 0.25;
  p.thresholds.bump_entry_radius = 0.5;
  p.thresholds.bump_align_time_s = 0.5;
  p.thresholds.bump_timeout_s = 20.0;
  p.patrol = {{8.0, 0.0, 5.0}};           // far side of the segment
  p.supply = {-1.0, 0.0, 0.0};            // near side (retreat target)
  BumpSegment seg;
  seg.entry = {1.0, 0.0, 0.0};
  seg.exit  = {5.0, 0.0, 0.0};
  seg.yaw = 0.0;
  p.bump_segments = {seg};
  return p;
}

struct Harness
{
  std::vector<Waypoint> goals;
  std::vector<double>   vels;       // every bump velocity command (in order)
  bool nav_cancelled{false};
  Context ctx;
  DecisionFsm fsm;

  explicit Harness(Profile p)
  : fsm(
      std::move(p),
      [this](const Waypoint & w) { goals.push_back(w); },
      [this]() { nav_cancelled = true; },
      [this](double vx) { vels.push_back(vx); })
  {
    ctx.set_thresholds(bump_profile().thresholds);
    ctx.set_now(0.0);
  }

  // healthy, running, at a given position
  void healthy_at(double x, double y, double t)
  {
    ctx.set_now(t);
    ctx.update(game(RUNNING, 300));
    ctx.update(robot(FULL_HP, FULL_AMMO));
    ctx.set_sentry_position(x, y);
  }

  double last_vel() const { return vels.empty() ? 0.0 : vels.back(); }
};

}  // namespace


TEST(BumpTraverse, EntersWhenSegmentBetweenRobotAndGoal)
{
  Harness h{bump_profile()};
  h.healthy_at(0.0, 0.0, 0.0);
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::BUMP_TRAVERSE);
}

TEST(BumpTraverse, NoTriggerWhenGoalOnSameSide)
{
  Profile p = bump_profile();
  p.patrol = {{-2.0, 0.0, 5.0}};
  Harness h{std::move(p)};
  h.healthy_at(0.0, 0.0, 0.0);
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(BumpTraverse, NoTriggerWhenOutsideYCorridor)
{
  Harness h{bump_profile()};
  h.healthy_at(0.0, 5.0, 0.0);
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(BumpTraverse, NoTriggerWhenNoSegmentsConfigured)
{
  Profile p = bump_profile();
  p.bump_segments.clear();
  Harness h{std::move(p)};
  h.healthy_at(0.0, 0.0, 0.0);
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}


// Full flow: GOTO_ENTRY -> ALIGN -> DASHING(constant) -> DONE -> PATROL
TEST(BumpTraverse, FullFlowForwardCrossing)
{
  Harness h{bump_profile()};
  h.healthy_at(0.0, 0.0, 0.0);

  // GOTO_ENTRY: sends Nav2 goal to entry (x=1).
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::BUMP_TRAVERSE);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 1.0);

  // Arrive at entry -> cancel Nav2, enter ALIGN.
  h.healthy_at(1.0, 0.0, 0.5);
  h.ctx.set_nav_status(NavStatus::ARRIVED);
  h.fsm.tick(h.ctx, 0.5);
  EXPECT_TRUE(h.nav_cancelled);

  // ALIGN: zero velocity until align_time (0.5s) elapses.
  h.fsm.tick(h.ctx, 0.6);
  EXPECT_DOUBLE_EQ(h.last_vel(), 0.0);

  // DASHING: constant +dash_speed.
  h.healthy_at(1.0, 0.0, 1.1);
  h.fsm.tick(h.ctx, 1.1);
  EXPECT_DOUBLE_EQ(h.last_vel(), 1.0);

  // Mid-segment: still constant (no slowdown on washboard).
  h.healthy_at(3.0, 0.0, 2.0);
  h.fsm.tick(h.ctx, 2.0);
  EXPECT_DOUBLE_EQ(h.last_vel(), 1.0);

  // Cross exit x=5 within tol -> hard stop, DONE.
  h.healthy_at(5.0, 0.0, 3.0);
  h.fsm.tick(h.ctx, 3.0);
  EXPECT_DOUBLE_EQ(h.last_vel(), 0.0);

  // DONE settles for bump_stop_ticks zero-vel frames, then hands back to PATROL.
  for (double t = 3.1; t < 4.0; t += 0.1) h.fsm.tick(h.ctx, t);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}


// Helper: drive the FSM into DASHING and return the harness ready to dash.
static void drive_to_dashing(Harness & h)
{
  h.healthy_at(0.0, 0.0, 0.0);
  h.fsm.tick(h.ctx, 0.0);                 // GOTO_ENTRY, goal sent
  h.healthy_at(1.0, 0.0, 0.5);
  h.ctx.set_nav_status(NavStatus::ARRIVED);
  h.fsm.tick(h.ctx, 0.5);                 // -> ALIGN
  h.healthy_at(1.0, 0.0, 1.1);
  h.fsm.tick(h.ctx, 1.1);                 // -> DASHING
}

// Preempt-proof: hp drops to resupply level mid-dash -> must NOT leave.
TEST(BumpTraverse, DashingNotPreemptedByLowHp)
{
  Harness h{bump_profile()};
  drive_to_dashing(h);
  ASSERT_EQ(h.fsm.state(), State::BUMP_TRAVERSE);
  // hp crashes below hp_low mid-segment
  h.ctx.set_now(2.0);
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(50, FULL_AMMO));     // would normally force RESUPPLY
  h.ctx.set_sentry_position(3.0, 0.0);
  h.fsm.tick(h.ctx, 2.0);
  EXPECT_EQ(h.fsm.state(), State::BUMP_TRAVERSE)
      << "low hp must not interrupt an in-progress bump dash (stranded risk)";
  EXPECT_DOUBLE_EQ(h.last_vel(), 1.0);    // still dashing
}

// Only IDLE (game over / referee stale) may interrupt: emits zero velocity.
TEST(BumpTraverse, DashingInterruptedByGameOver)
{
  Harness h{bump_profile()};
  drive_to_dashing(h);
  ASSERT_EQ(h.fsm.state(), State::BUMP_TRAVERSE);
  // game ends mid-dash
  h.ctx.set_now(2.0);
  h.ctx.update(game(rm_interfaces::msg::GameStatus::GAME_OVER, 0));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.ctx.set_sentry_position(3.0, 0.0);
  h.fsm.tick(h.ctx, 2.0);
  EXPECT_EQ(h.fsm.state(), State::IDLE);
}

// Washboard TF loss: position invalid -> hold last velocity, do NOT stop.
TEST(BumpTraverse, BlindDashHoldsLastVelocityOnPositionLoss)
{
  Harness h{bump_profile()};
  drive_to_dashing(h);
  ASSERT_DOUBLE_EQ(h.last_vel(), 1.0);
  // localization drops out mid-dash
  h.ctx.set_now(2.0);
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.ctx.invalidate_sentry_position();
  h.fsm.tick(h.ctx, 2.0);
  EXPECT_DOUBLE_EQ(h.last_vel(), 1.0)
      << "on TF loss keep dashing at last velocity, never stop (trough-stall risk)";
  EXPECT_EQ(h.fsm.state(), State::BUMP_TRAVERSE);
}

// Timeout mid-dash -> stop, then reverse-crawl back toward entry.
TEST(BumpTraverse, DashTimeoutTriggersReverseRecovery)
{
  Profile p = bump_profile();
  p.thresholds.bump_timeout_s = 5.0;
  Harness h{std::move(p)};
  drive_to_dashing(h);
  ASSERT_EQ(h.fsm.state(), State::BUMP_TRAVERSE);
  // stuck at x=3 well past timeout
  h.healthy_at(3.0, 0.0, 20.0);
  h.fsm.tick(h.ctx, 20.0);
  // reverse crawl: negative velocity (back toward entry x=1 < 3)
  h.healthy_at(3.0, 0.0, 20.1);
  h.fsm.tick(h.ctx, 20.1);
  EXPECT_LT(h.last_vel(), 0.0)
      << "after dash timeout the sentry must reverse-crawl back toward entry";
}

// Constant speed: no end-of-segment slowdown (washboard needs momentum).
// This is the key difference vs the single-ramp reference design.
TEST(BumpTraverse, ConstantSpeedNoSlowdownNearExit)
{
  Harness h{bump_profile()};
  drive_to_dashing(h);
  // far from exit
  h.healthy_at(2.0, 0.0, 2.0);
  h.fsm.tick(h.ctx, 2.0);
  const double far = h.last_vel();
  // very close to exit (x=4.9, exit=5, tol=0.25) but not yet crossed... actually
  // within tol so it would stop; use 4.5 which is close but outside tol.
  h.healthy_at(4.5, 0.0, 2.5);
  h.fsm.tick(h.ctx, 2.5);
  const double near = h.last_vel();
  EXPECT_DOUBLE_EQ(far, near)
      << "speed must stay constant near the exit — no ramp-down on washboard";
  EXPECT_DOUBLE_EQ(near, 1.0);
}

// Backward crossing: robot on the high-x side, goal on the low side (retreat).
TEST(BumpTraverse, BackwardCrossingNegativeSpeed)
{
  Profile p = bump_profile();
  p.patrol = {{-2.0, 0.0, 5.0}};   // goal on low side
  Harness h{std::move(p)};
  // robot starts on the high side (x=8), must cross back to reach x=-2
  h.healthy_at(8.0, 0.0, 0.0);
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::BUMP_TRAVERSE);
  // entry target for backward crossing is the high side (exit pt x=5)
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 5.0);
  // arrive, align, dash — velocity should be negative (toward low x)
  h.healthy_at(5.0, 0.0, 0.5);
  h.ctx.set_nav_status(NavStatus::ARRIVED);
  h.fsm.tick(h.ctx, 0.5);
  h.healthy_at(5.0, 0.0, 1.1);
  h.fsm.tick(h.ctx, 1.1);
  EXPECT_LT(h.last_vel(), 0.0);
  EXPECT_DOUBLE_EQ(h.last_vel(), -1.0);
}


// DONE settles for exactly bump_stop_ticks frames before handing back (L1 fix).
TEST(BumpTraverse, DoneSettlesForConfiguredStopTicks)
{
  Profile p = bump_profile();
  p.thresholds.bump_stop_ticks = 3;
  Harness h{std::move(p)};
  drive_to_dashing(h);
  // reach exit -> DONE
  h.healthy_at(5.0, 0.0, 2.0);
  h.fsm.tick(h.ctx, 2.0);
  ASSERT_EQ(h.fsm.state(), State::BUMP_TRAVERSE);   // DONE but still settling
  // must stay BUMP_TRAVERSE while emitting the first stop frames
  h.healthy_at(5.0, 0.0, 2.1);
  h.fsm.tick(h.ctx, 2.1);
  EXPECT_EQ(h.fsm.state(), State::BUMP_TRAVERSE)
      << "must settle bump_stop_ticks frames before handing back";
  // after enough frames -> PATROL
  for (double t = 2.2; t < 3.0; t += 0.1) h.fsm.tick(h.ctx, t);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
