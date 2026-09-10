#include <capability_mission_planner/offline_planner_config.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace capability_mission_planner::offline {
namespace {

std::filesystem::path resolve_path(
  const std::filesystem::path& config_path,
  const std::string& value)
{
  std::filesystem::path path(value);
  if (path.is_relative()) path = config_path.parent_path() / path;
  return std::filesystem::absolute(path).lexically_normal();
}

void require_map(const YAML::Node& node, const std::string& name) {
  if (!node || !node.IsMap()) throw std::runtime_error(name + " must be a map");
}

CapabilitySet capabilities(const YAML::Node& node, const std::string& name) {
  if (!node || !node.IsSequence())
    throw std::runtime_error(name + " must be a sequence of strings");
  CapabilitySet result;
  for (const auto& item : node) {
    const auto value = item.as<std::string>();
    if (value.empty()) throw std::runtime_error(name + " contains an empty capability");
    result.insert(value);
  }
  return result;
}

bool navigation_checkpoint_type_is_valid(const std::string& value) {
  static const std::set<std::string> valid{
    "start", "task", "turn", "resource_entry", "resource_exit",
    "transition_entry", "transition_exit", "holding", "finish"};
  return valid.count(value) != 0U;
}

struct ParsedPosition {
  GridPosition grid;
  CoordinateRepresentation representation = CoordinateRepresentation::Grid;
};

ParsedPosition position(
  const YAML::Node& node,
  const MultiMapBundle& bundle,
  const std::string& name,
  double tolerance_m = 0.0)
{
  require_map(node, name);
  if (!node["map_id"]) throw std::runtime_error(name + ".map_id is required");
  const auto map_id = node["map_id"].as<std::string>();
  const auto& map = bundle.map(map_id);
  const int representations = static_cast<int>(!!node["grid"]) +
    static_cast<int>(!!node["local_xy"]) + static_cast<int>(!!node["root_xy"]);
  if (representations != 1)
    throw std::runtime_error(name + " must contain exactly one of grid, local_xy, root_xy");

  ParsedPosition parsed;
  if (node["grid"]) {
    const auto value = node["grid"];
    if (!value.IsSequence() || value.size() != 2U)
      throw std::runtime_error(name + ".grid must contain [x, y]");
    parsed.grid = {map_id, value[0].as<int>(), value[1].as<int>()};
    parsed.representation = CoordinateRepresentation::Grid;
  } else {
    const bool is_root = !!node["root_xy"];
    const char* key = is_root ? "root_xy" : "local_xy";
    const auto value = node[key];
    if (!value.IsSequence() || value.size() != 2U)
      throw std::runtime_error(name + "." + key + " must contain [x, y] in metres");
    MetricPose pose{map_id, value[0].as<double>(), value[1].as<double>()};
    if (is_root) pose = map.root_to_local(pose);
    parsed.grid = map.local_to_grid(pose.x, pose.y);
    parsed.representation = is_root ? CoordinateRepresentation::RootXY :
      CoordinateRepresentation::LocalXY;
  }

  GridPosition& result = parsed.grid;
  if (!bundle.traversable(result)) {
    if (tolerance_m <= 0.0)
      throw std::runtime_error(name + " resolves to a blocked or out-of-map cell");
    const int radius = static_cast<int>(std::ceil(
      tolerance_m / map.resolution));
    GridPosition best = result;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        GridPosition candidate{map_id, result.x + dx, result.y + dy};
        if (!bundle.traversable(candidate)) continue;
        const double distance = std::hypot(dx, dy) * map.resolution;
        if (distance <= tolerance_m + 1e-9 && distance < best_distance) {
          best = candidate;
          best_distance = distance;
        }
      }
    }
    if (!bundle.traversable(best))
      throw std::runtime_error(name + " has no traversable cell within tolerance");
    result = best;
  }
  return parsed;
}

void load_traversal(const YAML::Node& node, TraversalOptions& options) {
  if (!node || node.IsNull()) return;
  require_map(node, "planner.traversal");
  if (node["time_step_seconds"])
    options.time_step_seconds = node["time_step_seconds"].as<double>();
  if (node["nominal_speed_mps"])
    options.nominal_speed_mps = node["nominal_speed_mps"].as<double>();
  if (node["default_transition_seconds"])
    options.default_transition_seconds = node["default_transition_seconds"].as<double>();
  if (node["map_switch_seconds"])
    options.map_switch_seconds = node["map_switch_seconds"].as<double>();
  if (node["obstacle_cost_weight"])
    options.obstacle_cost_weight = node["obstacle_cost_weight"].as<double>();
  if (node["allow_diagonal"])
    options.allow_diagonal = node["allow_diagonal"].as<bool>();
  if (node["coarse_search_factor"])
    options.coarse_search_factor = node["coarse_search_factor"].as<unsigned int>();
  if (node["downsample_costmap"])
    options.downsample_costmap = node["downsample_costmap"].as<bool>();
  if (node["resource_buffer_seconds"])
    options.resource_buffer_seconds = node["resource_buffer_seconds"].as<double>();
  if (node["coordination_max_high_level_nodes"])
    options.coordination_max_high_level_nodes =
      node["coordination_max_high_level_nodes"].as<std::size_t>();
  if (node["transition_seconds"]) {
    require_map(node["transition_seconds"], "planner.traversal.transition_seconds");
    for (const auto& item : node["transition_seconds"])
      options.transition_seconds[item.first.as<std::string>()] = item.second.as<double>();
  }
  if (node["transition_requirements"]) {
    require_map(node["transition_requirements"],
      "planner.traversal.transition_requirements");
    for (const auto& item : node["transition_requirements"]) {
      const auto type = item.first.as<std::string>();
      options.transition_requirements[type] = capabilities(
        item.second, "transition requirement " + type);
    }
  }
  if (!(options.time_step_seconds > 0.0) || !(options.nominal_speed_mps > 0.0) ||
    options.default_transition_seconds < 0.0 || options.map_switch_seconds < 0.0 ||
    options.obstacle_cost_weight < 0.0 || options.coarse_search_factor == 0U ||
    options.resource_buffer_seconds < 0.0)
  {
    throw std::runtime_error("planner traversal times and speed are invalid");
  }
  for (const auto& [type, seconds] : options.transition_seconds)
    if (seconds < 0.0) throw std::runtime_error("negative transition time for " + type);
}

} // namespace

ConfiguredMission OfflinePlannerConfigLoader::load(
  const std::filesystem::path& raw_config_path)
{
  const auto config_path = std::filesystem::absolute(raw_config_path).lexically_normal();
  const auto root = YAML::LoadFile(config_path.string());
  return load_node(root, config_path);
}

ConfiguredMission OfflinePlannerConfigLoader::load_node(
  const YAML::Node& root,
  const std::filesystem::path& raw_base_path)
{
  const auto base_path = raw_base_path.empty() ?
    std::filesystem::current_path() / "inline_mission.yaml" :
    std::filesystem::absolute(raw_base_path).lexically_normal();
  require_map(root, "configuration root");
  require_map(root["map"], "map");
  if (!root["map"]["directory"])
    throw std::runtime_error("map.directory is required");
  MapLoadOptions map_options;
  if (root["map"]["allow_unknown"])
    map_options.allow_unknown = root["map"]["allow_unknown"].as<bool>();
  if (root["map"]["inflation_radius_m"])
    map_options.inflation_radius = root["map"]["inflation_radius_m"].as<double>();
  if (root["map"]["inscribed_radius_m"])
    map_options.inscribed_radius = root["map"]["inscribed_radius_m"].as<double>();
  if (root["map"]["cost_scaling_factor"])
    map_options.cost_scaling_factor = root["map"]["cost_scaling_factor"].as<double>();
  if (root["map"]["persistent_cache"])
    map_options.persistent_cache = root["map"]["persistent_cache"].as<bool>();
  if (root["map"]["cache_directory"])
    map_options.cache_directory = resolve_path(
      base_path, root["map"]["cache_directory"].as<std::string>());
  if (map_options.inflation_radius < 0.0)
    throw std::runtime_error("map.inflation_radius_m must be non-negative");
  if (map_options.inscribed_radius < 0.0 || map_options.cost_scaling_factor <= 0.0)
    throw std::runtime_error("map clearance cost parameters are invalid");

  ConfiguredMission result;
  result.bundle = MapBundleLoader::load(
    resolve_path(base_path, root["map"]["directory"].as<std::string>()), map_options);
  if (root["output_directory"])
    result.output_directory = resolve_path(
      base_path, root["output_directory"].as<std::string>());
  else
    result.output_directory = base_path.parent_path();

  const auto planner = root["planner"];
  if (planner) require_map(planner, "planner");
  load_traversal(planner ? planner["traversal"] : YAML::Node{}, result.traversal);
  if (planner && planner["shared_resources"]) {
    if (!planner["shared_resources"].IsSequence())
      throw std::runtime_error("planner.shared_resources must be a sequence");
    for (const auto& resource_node : planner["shared_resources"]) {
      require_map(resource_node, "planner.shared_resources item");
      if (!resource_node["id"] || !resource_node["cells"] ||
        !resource_node["cells"].IsSequence())
        throw std::runtime_error("shared resource requires id and cells");
      SharedResource resource;
      resource.id = resource_node["id"].as<std::string>();
      resource.capacity = resource_node["capacity"] ?
        resource_node["capacity"].as<std::size_t>() : 1U;
      resource.buffer_seconds = resource_node["buffer_seconds"] ?
        resource_node["buffer_seconds"].as<double>() : result.traversal.resource_buffer_seconds;
      if (resource.id.empty() || resource.capacity != 1U || resource.buffer_seconds < 0.0)
        throw std::runtime_error("invalid shared resource parameters");
      for (const auto& cell : resource_node["cells"]) {
        if (!cell.IsSequence() || cell.size() != 3U)
          throw std::runtime_error("shared resource cells must be [map_id, x, y]");
        resource.cells.push_back({cell[0].as<std::string>(), cell[1].as<int>(), cell[2].as<int>()});
      }
      result.traversal.shared_resources.push_back(std::move(resource));
    }
  }
  if (planner && planner["objective"]) {
    require_map(planner["objective"], "planner.objective");
    const auto objective = planner["objective"];
    if (objective["maximum_load_weight"])
      result.objective.maximum_load = objective["maximum_load_weight"].as<double>();
    if (objective["total_load_weight"])
      result.objective.total_load = objective["total_load_weight"].as<double>();
    if (result.objective.maximum_load < 0.0 || result.objective.total_load < 0.0)
      throw std::runtime_error("planner objective weights must be non-negative");
  }
  if (planner && planner["coordinate_conflicts"])
    result.coordinate_conflicts = planner["coordinate_conflicts"].as<bool>();

  if (root["export"]) {
    require_map(root["export"], "export");
    if (root["export"]["path_thickness"])
      result.export_options.path_thickness = root["export"]["path_thickness"].as<int>();
    if (root["export"]["draw_grid"])
      result.export_options.draw_grid = root["export"]["draw_grid"].as<bool>();
    if (root["export"]["navigation_checkpoint_types"]) {
      const auto types = root["export"]["navigation_checkpoint_types"];
      if (!types.IsSequence())
        throw std::runtime_error("export.navigation_checkpoint_types must be a sequence");
      result.export_options.filter_navigation_checkpoint_types = true;
      for (const auto& type : types) {
        if (!type.IsScalar())
          throw std::runtime_error("export.navigation_checkpoint_types must contain strings");
        const auto value = type.as<std::string>();
        if (!navigation_checkpoint_type_is_valid(value))
          throw std::runtime_error("unknown navigation checkpoint type: " + value);
        result.export_options.navigation_checkpoint_types.insert(value);
      }
    }
    if (result.export_options.path_thickness <= 0)
      throw std::runtime_error("export.path_thickness must be positive");
  }

  if (!root["robots"] || !root["robots"].IsSequence())
    throw std::runtime_error("robots must be a sequence");
  for (std::size_t i = 0; i < root["robots"].size(); ++i) {
    const auto node = root["robots"][i];
    require_map(node, "robot");
    if (!node["id"] || !node["start"] || !node["capabilities"])
      throw std::runtime_error("each robot requires id, start, and capabilities");
    const auto start = position(node["start"], *result.bundle, "robot start");
    std::optional<GridPosition> return_position;
    if (node["return_home"]) {
      if (!node["return_home"].IsMap())
        throw std::runtime_error("robot return_home must be a coordinate map");
      return_position = position(
        node["return_home"], *result.bundle, "robot return_home").grid;
    }
    result.robots.push_back({node["id"].as<std::string>(),
      start.grid,
      capabilities(node["capabilities"], "robot capabilities"),
      return_position,
      node["clearance_radius_m"] ? node["clearance_radius_m"].as<double>() : 0.0,
      node["safety_margin_m"] ? node["safety_margin_m"].as<double>() : 0.0,
      node["nominal_speed_mps"] ? node["nominal_speed_mps"].as<double>() : 0.0,
      node["footprint_radius_m"] ? node["footprint_radius_m"].as<double>() : 0.0,
      start.representation});
    if (result.robots.back().clearance_radius_m < 0.0 ||
      result.robots.back().safety_margin_m < 0.0 ||
      result.robots.back().nominal_speed_mps < 0.0 ||
      result.robots.back().footprint_radius_m < 0.0)
      throw std::runtime_error("robot navigation profile values must be non-negative");
  }

  if (!root["tasks"] || !root["tasks"].IsSequence())
    throw std::runtime_error("tasks must be a sequence");
  for (std::size_t i = 0; i < root["tasks"].size(); ++i) {
    const auto node = root["tasks"][i];
    require_map(node, "task");
    if (!node["id"] || !node["location"] || !node["requirements"] ||
      !node["category"] || !node["service_seconds"])
    {
      throw std::runtime_error(
        "each task requires id, location, requirements, category, and service_seconds");
    }
    const double tolerance = node["position_tolerance_m"] ?
      node["position_tolerance_m"].as<double>() :
      (node["location"]["position_tolerance_m"] ?
        node["location"]["position_tolerance_m"].as<double>() : 0.0);
    if (tolerance < 0.0) throw std::runtime_error("task position tolerance must be non-negative");
    const auto location = position(
      node["location"], *result.bundle, "task location", tolerance);
    auto task = make_mapped_task(
      node["id"].as<std::string>(),
      location.grid,
      capabilities(node["requirements"], "task requirements"),
      node["category"].as<std::string>(), node["service_seconds"].as<int>(),
      node["high_priority"] ? node["high_priority"].as<bool>() : false);
    task.position_tolerance_m = tolerance;
    task.coordinate_representation = location.representation;
    result.tasks.push_back(std::move(task));
  }
  return result;
}

} // namespace capability_mission_planner::offline
