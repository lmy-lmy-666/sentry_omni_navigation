// Copyright 2026 Boombroke
//
// Simplified YAML profile loader.
//
#include "omni_decision_sample/profile.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace omni_decision_sample
{
namespace
{

Waypoint parse_waypoint(const YAML::Node & node)
{
  Waypoint wp;
  wp.x = node["x"].as<double>();
  wp.y = node["y"].as<double>();
  if (node["dwell_s"]) wp.dwell_s = node["dwell_s"].as<double>();

  if (!std::isfinite(wp.x) || !std::isfinite(wp.y) || !std::isfinite(wp.dwell_s)) {
    throw std::runtime_error(
      "waypoint contains NaN/Inf: (" + std::to_string(wp.x) + ", " +
      std::to_string(wp.y) + ", " + std::to_string(wp.dwell_s) + ")");
  }
  return wp;
}

Route parse_route(const YAML::Node & node)
{
  Route route;
  if (!node || !node.IsSequence()) return route;
  for (const auto & item : node) route.push_back(parse_waypoint(item));
  return route;
}

template <typename T>
T get_or(const YAML::Node & node, const char * key, T fallback)
{
  return (node && node[key]) ? node[key].template as<T>() : fallback;
}

BumpSegment parse_bump_segment(const YAML::Node & node, double y_tol)
{
  BumpSegment seg;
  if (!node["entry"] || !node["exit"]) {
    throw std::runtime_error("bump segment requires both 'entry' and 'exit'");
  }
  seg.entry = parse_waypoint(node["entry"]);
  seg.exit  = parse_waypoint(node["exit"]);
  if (node["yaw"]) seg.yaw = node["yaw"].as<double>();

  if (!std::isfinite(seg.yaw)) {
    throw std::runtime_error("bump segment yaw is NaN/Inf");
  }
  // Axis-aligned-in-X constraint: the swerve chassis crosses as a pure linear.x
  // dash, so entry and exit must share (near-)identical y.
  if (std::abs(seg.entry.y - seg.exit.y) > y_tol) {
    throw std::runtime_error(
      "bump segment not X-aligned: |entry.y - exit.y| = " +
      std::to_string(std::abs(seg.entry.y - seg.exit.y)) + " > bump_y_tol " +
      std::to_string(y_tol));
  }
  // Must have real X extent, else there is nothing to dash across.
  if (std::abs(seg.exit.x - seg.entry.x) < 1e-3) {
    throw std::runtime_error("bump segment has ~zero X extent (entry.x == exit.x)");
  }
  return seg;
}

}  // namespace

Profile load_profile(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("failed to load profile '" + yaml_path + "': " + e.what());
  }

  Profile p;
  p.name = get_or<std::string>(root, "name", "unnamed");

  // --- thresholds (YAML overrides defaults) -------------------------
  const Thresholds def;
  const YAML::Node th = root["thresholds"];
  p.thresholds.max_hp           = get_or<uint16_t>(th, "max_hp", def.max_hp);
  p.thresholds.hp_low           = get_or<uint16_t>(th, "hp_low", def.hp_low);
  p.thresholds.ammo_low         = get_or<uint16_t>(th, "ammo_low", def.ammo_low);
  p.thresholds.ammo_ok          = get_or<uint16_t>(th, "ammo_ok", def.ammo_ok);
  p.thresholds.game_total_time  = get_or<int32_t>(th, "game_total_time", def.game_total_time);
  p.thresholds.min_ticks_in_state = get_or<uint8_t>(th, "min_ticks_in_state", def.min_ticks_in_state);
  p.thresholds.stuck_timeout_s     = get_or<double>(th, "stuck_timeout_s", def.stuck_timeout_s);
  p.thresholds.resupply_timeout_s  = get_or<double>(th, "resupply_timeout_s", def.resupply_timeout_s);
  p.thresholds.referee_stale_timeout_s = get_or<double>(th, "referee_stale_timeout_s", def.referee_stale_timeout_s);
  p.thresholds.opening_strike_duration_s = get_or<double>(th, "opening_strike_duration_s", def.opening_strike_duration_s);
  p.thresholds.bump_dash_speed    = get_or<double>(th, "bump_dash_speed", def.bump_dash_speed);
  p.thresholds.bump_reverse_speed = get_or<double>(th, "bump_reverse_speed", def.bump_reverse_speed);
  p.thresholds.bump_tol           = get_or<double>(th, "bump_tol", def.bump_tol);
  p.thresholds.bump_entry_radius  = get_or<double>(th, "bump_entry_radius", def.bump_entry_radius);
  p.thresholds.bump_y_tol         = get_or<double>(th, "bump_y_tol", def.bump_y_tol);
  p.thresholds.bump_align_time_s  = get_or<double>(th, "bump_align_time_s", def.bump_align_time_s);
  p.thresholds.bump_timeout_s     = get_or<double>(th, "bump_timeout_s", def.bump_timeout_s);
  p.thresholds.bump_stop_ticks    = get_or<int>(th, "bump_stop_ticks", def.bump_stop_ticks);

  // --- routes & waypoints -------------------------------------------
  p.patrol          = parse_route(root["patrol"]);
  p.patrol_aggressive = parse_route(root["patrol_aggressive"]);
  if (root["supply"]) p.supply = parse_waypoint(root["supply"]);

  if (root["opening_strike"]) {
    p.opening_strike = parse_waypoint(root["opening_strike"]);
    p.has_opening_strike = true;
  }

  if (root["backup_supply_points"])
    p.backup_supply_points = parse_route(root["backup_supply_points"]);

  // --- undulating terrain segments ----------------------------------
  if (root["bump_segments"] && root["bump_segments"].IsSequence()) {
    for (const auto & item : root["bump_segments"]) {
      p.bump_segments.push_back(parse_bump_segment(item, p.thresholds.bump_y_tol));
    }
  }

  // --- validation --------------------------------------------------
  if (p.patrol.empty()) {
    throw std::runtime_error("profile '" + yaml_path + "': 'patrol' route is required");
  }

  return p;
}

}  // namespace omni_decision_sample
