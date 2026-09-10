#include <capability_mission_planner/offline_map_planner.hpp>

#include <algorithm>
#include <deque>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

using namespace capability_mission_planner;
using namespace capability_mission_planner::offline;

namespace {

std::vector<GridPosition> component(
  const MultiMapBundle& bundle,
  const std::string& map_id,
  GridPosition anchor)
{
  const auto& map = bundle.map(map_id);
  std::vector<unsigned char> visited(static_cast<std::size_t>(map.width * map.height));
  std::deque<GridPosition> queue{anchor};
  visited[static_cast<std::size_t>(anchor.y * map.width + anchor.x)] = 1U;
  std::vector<GridPosition> result;
  static constexpr int offsets[4][2]{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  while (!queue.empty()) {
    const auto current = queue.front();
    queue.pop_front();
    result.push_back(current);
    for (const auto& offset : offsets) {
      GridPosition next{map_id, current.x + offset[0], current.y + offset[1]};
      if (!bundle.traversable(next))
        continue;
      const auto index = static_cast<std::size_t>(next.y * map.width + next.x);
      if (visited[index] == 0U) {
        visited[index] = 1U;
        queue.push_back(next);
      }
    }
  }
  return result;
}

GridPosition first_free(const MapLayer& map) {
  for (int y = 0; y < map.height; ++y)
    for (int x = 0; x < map.width; ++x)
      if (map.is_traversable(x, y)) return {map.id, x, y};
  throw std::runtime_error("map has no free cells: " + map.id);
}

std::vector<GridPosition> map_component(
  const MultiMapBundle& bundle,
  const std::string& map_id)
{
  for (const auto& transition : bundle.transitions)
    if (transition.from_map == map_id)
      return component(bundle, map_id, transition.from_cell);

  const auto& map = bundle.map(map_id);
  std::vector<unsigned char> visited(static_cast<std::size_t>(map.width * map.height));
  std::vector<GridPosition> largest;
  static constexpr int offsets[4][2]{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      const auto seed_index = static_cast<std::size_t>(y * map.width + x);
      if (visited[seed_index] != 0U || !map.is_traversable(x, y))
        continue;
      std::deque<GridPosition> queue{{map_id, x, y}};
      visited[seed_index] = 1U;
      std::vector<GridPosition> current;
      while (!queue.empty()) {
        const auto cell = queue.front();
        queue.pop_front();
        current.push_back(cell);
        for (const auto& offset : offsets) {
          GridPosition next{map_id, cell.x + offset[0], cell.y + offset[1]};
          if (!bundle.traversable(next))
            continue;
          const auto index = static_cast<std::size_t>(next.y * map.width + next.x);
          if (visited[index] == 0U) {
            visited[index] = 1U;
            queue.push_back(next);
          }
        }
      }
      if (current.size() > largest.size())
        largest = std::move(current);
    }
  }
  if (largest.empty())
    return component(bundle, map_id, first_free(map));
  return largest;
}

std::vector<GridPosition> spread_points(
  const MultiMapBundle& bundle,
  const std::string& map_id,
  std::size_t count)
{
  auto cells = map_component(bundle, map_id);
  std::vector<GridPosition> candidates;
  candidates.reserve(cells.size());
  const auto& map = bundle.map(map_id);
  for (const auto& cell : cells) {
    bool clearance = true;
    for (int dy = -2; dy <= 2 && clearance; ++dy)
      for (int dx = -2; dx <= 2; ++dx)
        clearance = clearance && map.is_traversable(cell.x + dx, cell.y + dy);
    if (clearance)
      candidates.push_back(cell);
  }
  if (candidates.size() < count)
    candidates = std::move(cells);
  if (candidates.size() < count)
    throw std::runtime_error("not enough connected free cells in " + map_id);

  std::vector<GridPosition> selected;
  selected.push_back(candidates[candidates.size() / 2U]);
  while (selected.size() < count) {
    int best_distance = -1;
    GridPosition best;
    for (const auto& candidate : candidates) {
      if (std::find(selected.begin(), selected.end(), candidate) != selected.end())
        continue;
      int nearest = std::numeric_limits<int>::max();
      for (const auto& existing : selected)
        nearest = std::min(nearest,
          std::abs(candidate.x - existing.x) + std::abs(candidate.y - existing.y));
      if (nearest > best_distance) {
        best_distance = nearest;
        best = candidate;
      }
    }
    selected.push_back(best);
  }
  return selected;
}

} // namespace

int main(int argc, char* argv[]) {
  try {
    if (argc < 3) {
      std::cerr << "usage: offline_map_demo MAP_DIRECTORY OUTPUT_DIRECTORY [--no-cbs]\n";
      return 2;
    }
    const bool coordinate = argc < 4 || std::string(argv[3]) != "--no-cbs";
    const auto bundle = MapBundleLoader::load(argv[1]);
    std::vector<std::string> map_ids;
    for (const auto& [id, map] : bundle->maps) {
      (void)map;
      map_ids.push_back(id);
    }
    const std::string first = map_ids.front();
    const std::string middle = map_ids[map_ids.size() / 2U];
    const std::string last = map_ids.back();
    const auto first_points = spread_points(*bundle, first,
      bundle->maps.size() == 1U ? 7U : 5U);
    const std::vector<GridPosition> starts{
      first_points[0], first_points[1], first_points[2]};
    const auto middle_points = middle == first
      ? std::vector<GridPosition>{first_points[4], first_points[5]}
      : spread_points(*bundle, middle, 2);
    const auto last_points = last == first
      ? std::vector<GridPosition>{first_points[5], first_points[6]}
      : spread_points(*bundle, last, 2);

    std::vector<MappedRobot> robots{
      {"a", starts[0], {"fire", "camera", "stairs"}, starts[0]},
      {"b", starts[1], {"camera", "thermal", "stairs"}, starts[1]},
      {"c", starts[2], {"fire", "camera", "thermal", "stairs"}, starts[2]}};
    std::vector<MappedTask> tasks{
      make_mapped_task("T1-photo", first_points[3], {"camera"}, "gimbal_photo", 2, true),
      make_mapped_task("T2-fire", first_points[3], {"fire"}, "fire_suppression", 4),
      make_mapped_task("T3-thermal", middle_points[0], {"thermal"}, "thermal_inspection", 3),
      make_mapped_task("T4-photo", middle_points[1], {"camera"}, "gimbal_photo", 2),
      make_mapped_task("T5-fire", last_points[0], {"fire"}, "fire_suppression", 4, true),
      make_mapped_task("T6-fire-photo", last_points[1], {"fire", "camera"},
        "fire_documentation", 5)};

    TraversalOptions traversal;
    MultiMapPathPlanner path_planner(bundle, traversal);
    OfflineMissionPlanner planner(std::move(path_planner));
    const auto plan = planner.plan(robots, tasks, coordinate);
    PlanExporter::write(argv[2], *bundle, robots, tasks, plan);

    std::cout << "PASS maps=" << bundle->maps.size() << " robots=" << robots.size()
              << " tasks=" << tasks.size() << " maximum_load_seconds="
              << plan.maximum_load_ticks * plan.time_step_seconds
              << " total_load_seconds="
              << plan.total_load_ticks * plan.time_step_seconds
              << " output=" << argv[2] << '\n';
    for (const auto& route : plan.routes) {
      std::cout << "  " << robots[route.robot_index].id << ": stops="
                << route.stops.size() << " travel_seconds="
                << route.travel_ticks * plan.time_step_seconds
                << " service_seconds="
                << route.service_ticks * plan.time_step_seconds << '\n';
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "offline map planning failed: " << error.what() << '\n';
    return 1;
  }
}
