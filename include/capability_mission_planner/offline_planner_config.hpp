#pragma once

#include <capability_mission_planner/offline_map_planner.hpp>

#include <filesystem>
#include <memory>
#include <vector>

namespace YAML {
class Node;
}

namespace capability_mission_planner::offline {

struct ConfiguredMission {
  std::shared_ptr<const MultiMapBundle> bundle;
  std::filesystem::path output_directory;
  TraversalOptions traversal;
  ObjectiveWeights objective;
  ExportOptions export_options;
  bool coordinate_conflicts = true;
  std::vector<MappedRobot> robots;
  std::vector<MappedTask> tasks;
};

class OfflinePlannerConfigLoader {
public:
  static ConfiguredMission load(const std::filesystem::path& config_path);
  static ConfiguredMission load_node(
    const YAML::Node& root,
    const std::filesystem::path& base_path = {},
    bool require_output_directory = false);
};

} // namespace capability_mission_planner::offline
