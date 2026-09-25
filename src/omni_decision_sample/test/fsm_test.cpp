// Copyright 2026 Boombroke
//
// Unit tests for simplified DecisionFsm (3 states: IDLE / PATROL / RESUPPLY).
//
#include "omni_decision_sample/fsm.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "omni_decision_sample/context.hpp"
#include "omni_decision_sample/profile.hpp"
#include "omni_decision_sample/types.hpp"

using omni_decision_sample::Context;
using omni_decision_sample::DecisionFsm;
using omni_decision_sample::NavStatus;
using omni_decision_sample::Profile;
using omni_decision_sample::State;
using omni_decision_sample::Waypoint;

namespace
{

Profile make_profile()
{
  Profile p;
  p.name = "test";
  p.thresholds.hp_low = 150;
  p.thresholds.ammo_low = 50;
  p.thresholds.ammo_ok = 100;
  p.thresholds.game_total_time = 420;
  p.thresholds.min_ticks_in_state = 1;   // fast tests
  p.thresholds.resupply_timeout_s = 30.0;
  p.patrol = {{1.0, 1.0, 5.0}, {2.0, 2.0, 5.0}};
  p.supply = {-1.0, -5.0, 0.0};
  return p;
}

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

rm_interfaces::msg::GameRobotHP outpost_hp(uint16_t hp)
{
  rm_interfaces::msg::GameRobotHP m;
  m.ally_outpost_hp = hp;
  return m;
}

struct Harness
{
  std::vector<Waypoint> goals;
  bool nav_cancelled{false};
  Context ctx;
  DecisionFsm fsm;

  explicit Harness(Profile p)
  : fsm(
      std::move(p),
      [this](const Waypoint & w) { goals.push_back(w); },
      [this]() { nav_cancelled = true; })
  {
    ctx.set_thresholds(make_profile().thresholds);
    ctx.set_now(0.0);
  }
};

constexpr uint8_t RUNNING = rm_interfaces::msg::GameStatus::RUNNING;

// "healthy" = full hp + plenty of ammo → PATROL
constexpr uint16_t FULL_HP = 400;
constexpr uint16_t FULL_AMMO = 300;

}  // namespace

// =============================================================================
//  Basic state transitions
// =============================================================================

TEST(DecisionFsm, IdleWhenGameNotRunning)
{
  Harness h{make_profile()};
  h.ctx.update(game(rm_interfaces::msg::GameStatus::PREPARATION, 400));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::IDLE);
}

TEST(DecisionFsm, PatrolByDefault)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, ResupplyWhenAmmoLow)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, 50));  // ammo == ammo_low → enter
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, ResupplyWhenHpLow)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));  // hp < hp_low
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, GameEndForcesIdle)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  h.ctx.update(game(rm_interfaces::msg::GameStatus::GAME_OVER, 0));
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::IDLE);
}

TEST(DecisionFsm, RefereeStaleForcesIdle)
{
  Harness h{make_profile()};
  h.ctx.set_now(0.0);
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  h.ctx.set_now(4.0);
  h.fsm.tick(h.ctx, 4.0);
  EXPECT_EQ(h.fsm.state(), State::IDLE);
}

// =============================================================================
//  Patrol routing
// =============================================================================

TEST(DecisionFsm, PatrolSendsFirstWaypoint)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  ASSERT_EQ(h.goals.size(), 1u);
  EXPECT_DOUBLE_EQ(h.goals[0].x, 1.0);
  EXPECT_DOUBLE_EQ(h.goals[0].y, 1.0);
}

TEST(DecisionFsm, PatrolAdvancesAfterArrivalAndDwell)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  h.ctx.set_nav_status(NavStatus::ARRIVED);
  h.fsm.tick(h.ctx, 1.0);
  h.fsm.tick(h.ctx, 6.0);
  h.ctx.set_nav_status(NavStatus::MOVING);
  h.fsm.tick(h.ctx, 6.0);
  ASSERT_GE(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, 2.0);
}

TEST(DecisionFsm, PatrolDoesNotAdvanceBeforeArrival)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  h.fsm.tick(h.ctx, 100.0);  // stuck timeout triggers skip
  EXPECT_EQ(h.goals.size(), 1u);
}

TEST(DecisionFsm, NavStuckAdvancesPatrol)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  const auto first = h.goals.back();
  h.ctx.set_nav_status(NavStatus::FAILED);
  h.fsm.tick(h.ctx, 1.0);
  h.ctx.set_nav_status(NavStatus::MOVING);
  h.fsm.tick(h.ctx, 2.0);
  EXPECT_GE(h.goals.size(), 2u);
  EXPECT_NE(h.goals.back().x, first.x);
}

// =============================================================================
//  Patrol tactical variants
// =============================================================================

TEST(DecisionFsm, PatrolUsesAggressiveRouteWhenOutpostAlive)
{
  Profile p = make_profile();
  p.patrol_aggressive = {{8.0, 0.0, 5.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.ctx.update(outpost_hp(500));  // outpost alive
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 8.0);
}

TEST(DecisionFsm, PatrolFallsBackToDefaultWhenOutpostDestroyed)
{
  // Outpost destroyed → must use default patrol (我方半场防守), not aggressive.
  Profile p = make_profile();
  p.patrol = {{1.0, 1.0, 5.0}};
  p.patrol_aggressive = {{8.0, 0.0, 5.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.ctx.update(outpost_hp(0));  // outpost destroyed
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 1.0);  // default route
}

TEST(DecisionFsm, PatrolResetsTrackingOnRouteSwitch)
{
  // Switching aggressive → default must reset path_idx_/goal_sent_ so we
  // republish from the new route's start instead of chasing a stale goal.
  Profile p = make_profile();
  p.patrol = {{1.0, 1.0, 5.0}, {2.0, 2.0, 5.0}, {3.0, 3.0, 5.0}};
  p.patrol_aggressive = {{8.0, 0.0, 5.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.ctx.update(outpost_hp(500));  // outpost alive → aggressive
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals.back().x, 8.0);  // aggressive point

  // outpost destroyed → switch to default; must republish from default[0]
  h.ctx.update(outpost_hp(0));
  h.ctx.set_nav_status(NavStatus::MOVING);
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_DOUBLE_EQ(h.goals.back().x, 1.0)
      << "route switch must reset tracking and republish from new route start";
}

// =============================================================================
//  Resupply — enter / exit hysteresis
// =============================================================================

TEST(DecisionFsm, ResupplyExitsOnlyWhenFullyRecovered)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));  // hp low → enter
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  h.ctx.update(robot(FULL_HP, FULL_AMMO));  // hp == max AND ammo ok
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, ResupplyStaysUntilHpFull)
{
  // hp recovered above hp_low but not yet full → must stay
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  h.ctx.update(robot(399, FULL_AMMO));  // one below max → still healing
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, ResupplyStaysUntilAmmoOk)
{
  // hp full but ammo still below ammo_ok → must stay (waiting for +100/min)
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, 40));  // ammo low → enter
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  h.ctx.update(robot(FULL_HP, 99));  // ammo below ammo_ok(100)
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
  h.ctx.update(robot(FULL_HP, 100));  // ammo == ammo_ok → leave
  h.fsm.tick(h.ctx, 2.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, ResupplySendsGoalToSupplyPad)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, -1.0);
  EXPECT_DOUBLE_EQ(h.goals[0].y, -5.0);
}

TEST(DecisionFsm, ResupplyStaysPutOnPositionArrivalWithoutRfid)
{
  // "At the supply pad" is judged by REAL-TIME position (within
  // supply_arrival_radius of the pad), NOT by a one-shot Nav2 arrival latch and
  // NOT by RFID (the upstream serial driver may mis-map the RFID bit).
  // While physically on the pad, the sentry stays put (no goal re-issue / no
  // 30s jitter) and heals.
  Profile p = make_profile();
  p.thresholds.resupply_timeout_s = 5.0;
  const auto pad = p.supply;  // (-1.0, -5.0)
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  const std::size_t goals_after_first = h.goals.size();

  // Sentry is physically standing on the supply pad (RFID never fires).
  h.ctx.set_sentry_position(pad.x, pad.y);
  h.fsm.tick(h.ctx, 1.0);

  // Well past resupply_timeout_s: must NOT re-issue a goal (stays put healing)
  h.fsm.tick(h.ctx, 10.0);
  h.fsm.tick(h.ctx, 20.0);
  EXPECT_EQ(h.goals.size(), goals_after_first)
      << "while physically on the pad the sentry must stay put, not keep "
         "re-issuing supply goals every timeout";
}

TEST(DecisionFsm, ResupplyReNavigatesWhenPushedOffPad)
{
  // Regression: sentry reached the pad, then was pushed OFF it (or killed and
  // respawned away from it). It must notice it is no longer on the pad and
  // re-navigate home — NOT heal in place off-pad forever. The old code latched
  // goal_arrived_ and returned unconditionally, stalling off-pad.
  Profile p = make_profile();
  p.thresholds.resupply_timeout_s = 100.0;  // large: prove re-issue is position-driven, not timeout
  const auto pad = p.supply;                // (-1.0, -5.0)
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));      // low hp → RESUPPLY

  // Drive to and settle on the pad.
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  h.ctx.set_sentry_position(pad.x, pad.y);
  h.fsm.tick(h.ctx, 1.0);
  const std::size_t goals_on_pad = h.goals.size();

  // Pushed well off the pad (still low hp, still RESUPPLY).
  h.ctx.set_sentry_position(pad.x + 3.0, pad.y + 3.0);
  h.fsm.tick(h.ctx, 2.0);

  EXPECT_GT(h.goals.size(), goals_on_pad)
      << "after being pushed off the pad the sentry must re-issue a supply goal "
         "and drive back, not heal in place off-pad";
  EXPECT_DOUBLE_EQ(h.goals.back().x, pad.x);
  EXPECT_DOUBLE_EQ(h.goals.back().y, pad.y);
}

TEST(DecisionFsm, BackupSupplyRotatedAfterTimeout)
{
  Profile p = make_profile();
  p.backup_supply_points = {{-3.0, -5.0, 0.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.goals.size(), 1u);
  EXPECT_DOUBLE_EQ(h.goals[0].x, -1.0);   // primary
  h.fsm.tick(h.ctx, 31.0);  // > resupply_timeout_s (30) → rotate
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, -3.0);   // backup
}

TEST(DecisionFsm, SupplyTargetCyclesBackToPrimary)
{
  // With one backup, rotation must cycle: primary → backup → primary → …
  // proving the sentry never permanently gives up on a supply point.
  Profile p = make_profile();
  p.backup_supply_points = {{-3.0, -5.0, 0.0}};
  p.thresholds.resupply_timeout_s = 1.0;
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.goals.size(), 1u);
  EXPECT_DOUBLE_EQ(h.goals[0].x, -1.0);   // primary
  h.fsm.tick(h.ctx, 2.0);
  EXPECT_DOUBLE_EQ(h.goals.back().x, -3.0);   // backup
  h.fsm.tick(h.ctx, 4.0);
  EXPECT_DOUBLE_EQ(h.goals.back().x, -1.0);   // back to primary
}

TEST(DecisionFsm, ResupplyRotatesImmediatelyOnNavFail)
{
  Profile p = make_profile();
  p.backup_supply_points = {{-3.0, -5.0, 0.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.goals.size(), 1u);
  h.ctx.set_nav_status(NavStatus::FAILED);
  h.fsm.tick(h.ctx, 1.0);
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, -3.0);
}

// =============================================================================
//  Respawn — sentry must keep heading home after reviving (never gives up)
// =============================================================================

TEST(DecisionFsm, RespawnKeepsNavigatingHome)
{
  // Sentry dies (hp 0) mid-match, stays "dead" far longer than any old
  // timeout cap, then revives. It must still be in RESUPPLY and still be
  // publishing goals to the supply pad — never stalled.
  // Referee data streams continuously, so we refresh it at every time step
  // (otherwise referee_fresh() would go stale and force IDLE).
  auto step = [](Harness & h, double t, uint16_t hp) {
    h.ctx.set_now(t);
    h.ctx.update(game(RUNNING, 300));
    h.ctx.update(robot(hp, FULL_AMMO));
    h.fsm.tick(h.ctx, t);
  };

  Harness h{make_profile()};
  step(h, 0.0, 0);   // dead
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  const std::size_t goals_before = h.goals.size();

  // long "dead" period — well past the removed 60/120 s caps
  step(h, 200.0, 0);

  // revive with low hp (respawn restores 10% → still below hp_low)
  step(h, 201.0, 40);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);

  // stuck timeout should still rotate/republish → new goals keep coming
  step(h, 240.0, 40);
  EXPECT_GT(h.goals.size(), goals_before)
      << "sentry must keep publishing supply goals after respawn, not stall";
}

// =============================================================================
//  Boundary cases
// =============================================================================

TEST(DecisionFsm, HpBoundaryNotEnterResupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(150, FULL_AMMO));  // == hp_low (not <)
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, HpBoundaryEnterResupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(149, FULL_AMMO));  // < hp_low
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, AmmoBoundaryEnterResupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, 51));  // > ammo_low → still patrol
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, IdleCancelsNavigation)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_FALSE(h.nav_cancelled);
  h.ctx.update(game(rm_interfaces::msg::GameStatus::PREPARATION, 400));
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_TRUE(h.nav_cancelled);
}

TEST(DecisionFsm, MidPatrolHpDropGoesResupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  h.ctx.update(robot(140, FULL_AMMO));  // hp drops below hp_low
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

// =============================================================================
//  Opening strike (kill enemy outpost at match start, once)
// =============================================================================

namespace
{
Profile strike_profile()
{
  Profile p = make_profile();
  p.opening_strike = {5.0, 0.0, 0.0};
  p.has_opening_strike = true;
  p.thresholds.opening_strike_duration_s = 90.0;
  return p;
}
}  // namespace

TEST(DecisionFsm, OpeningStrikeAtMatchStart)
{
  // With a firing spot configured, a healthy sentry goes to OPENING_STRIKE
  // first (not PATROL) and drives to the strike coordinate.
  Harness h{strike_profile()};
  h.ctx.update(game(RUNNING, 420));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::OPENING_STRIKE);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 5.0);
}

TEST(DecisionFsm, OpeningStrikeEndsAfterDwellThenPatrols)
{
  Harness h{strike_profile()};
  h.ctx.update(game(RUNNING, 420));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::OPENING_STRIKE);
  // still striking before dwell elapses
  h.fsm.tick(h.ctx, 89.0);
  EXPECT_EQ(h.fsm.state(), State::OPENING_STRIKE);
  // dwell elapsed → behave marks done, next tick selects PATROL
  h.fsm.tick(h.ctx, 90.0);
  h.fsm.tick(h.ctx, 91.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, OpeningStrikeOnlyOncePerMatch)
{
  // After the strike is consumed, dropping back to healthy must NOT re-trigger
  // another opening strike — it is a once-per-match action.
  Harness h{strike_profile()};
  h.ctx.update(game(RUNNING, 420));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::OPENING_STRIKE);
  h.fsm.tick(h.ctx, 91.0);   // dwell done
  h.fsm.tick(h.ctx, 92.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  // many ticks later, still healthy → must stay PATROL, never strike again
  h.fsm.tick(h.ctx, 200.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, ResupplyPreemptsOpeningStrikeAndConsumesIt)
{
  // If hp/ammo drops during the opening strike, RESUPPLY preempts it, and the
  // strike is consumed — after recovering, the sentry patrols, not re-strikes.
  Harness h{strike_profile()};
  h.ctx.update(game(RUNNING, 420));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::OPENING_STRIKE);

  // hp drops mid-strike → RESUPPLY preempts
  h.ctx.update(robot(100, FULL_AMMO));
  h.fsm.tick(h.ctx, 5.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);

  // fully recover → should PATROL (strike already consumed), not strike again
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 6.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, NoOpeningStrikeWhenNotConfigured)
{
  // Default profile has no opening_strike → behaves as before (straight to PATROL).
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 420));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
