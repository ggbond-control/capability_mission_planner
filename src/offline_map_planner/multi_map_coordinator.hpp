#pragma once

#include <capability_mission_planner/offline_map_planner.hpp>

namespace capability_mission_planner::offline {

std::vector<std::vector<TimedMapState>> coordinate_multi_map_routes(
  const MultiMapPathPlanner& path_planner,
  const std::vector<MappedRobot>& robots,
  const std::vector<MappedRobotRoute>& routes,
  CoordinationStats* stats = nullptr);

} // namespace capability_mission_planner::offline
