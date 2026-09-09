#include <capability_mission_planner/offline_map_planner.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace capability_mission_planner::offline {
namespace {

std::string json(const std::string& value) {
  std::ostringstream output;
  output << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character) << std::dec;
        } else {
          output << character;
        }
    }
  }
  output << '"';
  return output.str();
}

const char* checkpoint_type(NavigationCheckpointType type) {
  switch (type) {
    case NavigationCheckpointType::Start: return "start";
    case NavigationCheckpointType::Task: return "task";
    case NavigationCheckpointType::Turn: return "turn";
    case NavigationCheckpointType::ResourceEntry: return "resource_entry";
    case NavigationCheckpointType::ResourceExit: return "resource_exit";
    case NavigationCheckpointType::TransitionEntry: return "transition_entry";
    case NavigationCheckpointType::TransitionExit: return "transition_exit";
    case NavigationCheckpointType::Holding: return "holding";
    case NavigationCheckpointType::Finish: return "finish";
  }
  return "unknown";
}

void write_position(std::ostream& output, const GridPosition& position) {
  output << "\"map_id\": " << json(position.map_id)
    << ", \"grid\": [" << position.x << ", " << position.y << "]";
}

void write_annotations(std::ostream& output, const OfflineMissionPlan& plan,
  std::size_t robot, const ExportOptions& options)
{
  output << ",\n      \"navigation_checkpoints\": [";
  if (robot < plan.navigation_checkpoints.size()) {
    const auto& points = plan.navigation_checkpoints[robot];
    bool first = true;
    for (const auto& point : points) {
      const auto type = checkpoint_type(point.type);
      if (options.filter_navigation_checkpoint_types &&
        options.navigation_checkpoint_types.count(type) == 0U)
      {
        continue;
      }
      if (!first) output << ',';
      first = false;
      output << "\n        {\"type\": " << json(type)
        << ", \"id\": " << json(point.id)
        << ", \"arrival_tick\": " << point.arrival_tick
        << ", \"departure_tick\": " << point.departure_tick << ", ";
      write_position(output, point.position);
      if (!point.task_id.empty()) output << ", \"task\": " << json(point.task_id);
      if (!point.transition_id.empty())
        output << ", \"transition_id\": " << json(point.transition_id);
      if (!point.resource.empty()) output << ", \"resource\": " << json(point.resource);
      output << "}";
    }
  }
  output << "\n      ],\n      \"traffic_events\": [";
  if (robot < plan.traffic_events.size()) {
    const auto& events = plan.traffic_events[robot];
    for (std::size_t i = 0; i < events.size(); ++i) {
      const auto& event = events[i];
      if (i != 0U) output << ',';
      output << "\n        {\"type\": " << json(event.type)
        << ", \"checkpoint_id\": " << json(event.checkpoint_id)
        << ", \"start_tick\": " << event.start_tick
        << ", \"end_tick\": " << event.end_tick << ", ";
      write_position(output, event.position);
      if (!event.resource.empty()) output << ", \"resource\": " << json(event.resource);
      output << ", \"reason\": " << json(event.reason) << "}";
    }
  }
  output << "\n      ]";
}

cv::Point pixel(const MapLayer& map, const GridPosition& position) {
  return {position.x, map.height - 1 - position.y};
}

const std::vector<cv::Scalar>& colors() {
  static const std::vector<cv::Scalar> values{
    {35, 75, 220}, {45, 155, 35}, {210, 95, 30}, {165, 45, 170},
    {30, 165, 190}, {120, 80, 35}};
  return values;
}

std::string serialize_json(
  const MultiMapBundle& bundle,
  const std::vector<MappedRobot>& robots,
  const std::vector<MappedTask>& tasks,
  const OfflineMissionPlan& plan,
  const ExportOptions& options)
{
  std::ostringstream output;
  output << std::setprecision(10);
  output << "{\n  \"time_step_seconds\": " << plan.time_step_seconds
         << ",\n  \"maximum_load_ticks\": " << plan.maximum_load_ticks
         << ",\n  \"maximum_load_seconds\": "
         << plan.maximum_load_ticks * plan.time_step_seconds
         << ",\n  \"total_load_ticks\": " << plan.total_load_ticks
         << ",\n  \"total_load_seconds\": "
         << plan.total_load_ticks * plan.time_step_seconds << ",\n";
  output << "  \"maps\": [";
  std::size_t map_index = 0;
  for (const auto& [id, map] : bundle.maps) {
    if (map_index++ != 0U) output << ',';
    output << "\n    {\"id\": " << json(id) << ", \"width\": " << map.width
           << ", \"height\": " << map.height << ", \"resolution\": "
           << map.resolution << ", \"origin\": [" << map.origin_x << ", "
           << map.origin_y << ", " << map.origin_yaw << "], \"root_offset\": ["
           << map.root_x << ", " << map.root_y << ", " << map.root_yaw << "]}";
  }
  output << "\n  ],\n  \"routes\": [\n";
  for (std::size_t route_index = 0; route_index < plan.routes.size(); ++route_index) {
    const auto& route = plan.routes[route_index];
    const auto& robot = robots[route.robot_index];
    output << "    {\n      \"robot\": " << json(robot.id)
           << ",\n      \"travel_ticks\": " << route.travel_ticks
           << ",\n      \"service_ticks\": " << route.service_ticks
           << ",\n      \"load_seconds\": "
           << route.load_ticks() * plan.time_step_seconds << ",\n      \"stops\": [";
    for (std::size_t stop_index = 0; stop_index < route.stops.size(); ++stop_index) {
      const auto& stop = route.stops[stop_index];
      if (stop_index != 0U) output << ',';
      const auto local = bundle.map(stop.location.map_id).grid_to_local(stop.location);
      const auto root = bundle.map(stop.location.map_id).local_to_root(local);
      output << "\n        {\"map_id\": " << json(stop.location.map_id)
             << ", \"grid\": [" << stop.location.x << ", " << stop.location.y
             << "], \"local_xy\": [" << local.x << ", " << local.y
             << "], \"root_xy\": [" << root.x << ", " << root.y
             << "], \"position_tolerance_m\": " << stop.position_tolerance_m
             << ", \"service_ticks\": " << stop.service_ticks << ", \"tasks\": [";
      for (std::size_t i = 0; i < stop.task_indices.size(); ++i) {
        if (i != 0U) output << ", ";
        output << json(tasks[stop.task_indices[i]].id());
      }
      output << "]}";
    }
    output << "\n      ]";
    write_annotations(output, plan, route.robot_index, options);
    output << "\n    }";
    if (route_index + 1U != plan.routes.size()) output << ',';
    output << '\n';
  }
  output << "  ]\n}\n";
  return output.str();
}

void write_summary(
  const std::filesystem::path& path,
  const MultiMapBundle& bundle,
  const std::vector<MappedRobot>& robots,
  const std::vector<MappedTask>& tasks,
  const OfflineMissionPlan& plan)
{
  std::ofstream output(path);
  if (!output)
    throw std::runtime_error("cannot write " + path.string());
  output << "maps=" << bundle.maps.size() << " robots=" << robots.size()
         << " tasks=" << tasks.size() << '\n';
  output << "time_step_seconds=" << plan.time_step_seconds << '\n';
  output << "maximum_load_seconds="
         << plan.maximum_load_ticks * plan.time_step_seconds << '\n';
  output << "total_load_seconds="
         << plan.total_load_ticks * plan.time_step_seconds << "\n\n";
  for (const auto& route : plan.routes) {
    output << "robot " << robots[route.robot_index].id << ": travel="
           << route.travel_ticks * plan.time_step_seconds << "s service="
           << route.service_ticks * plan.time_step_seconds << "s stops="
           << route.stops.size() << '\n';
    for (const auto& stop : route.stops) {
      output << "  " << stop.location.map_id << " grid(" << stop.location.x
             << ',' << stop.location.y << ") tasks=";
      for (std::size_t i = 0; i < stop.task_indices.size(); ++i) {
        if (i != 0U) output << ',';
        output << tasks[stop.task_indices[i]].id();
      }
      output << '\n';
    }
  }
}

} // namespace

std::string PlanExporter::to_json(
  const MultiMapBundle& bundle,
  const std::vector<MappedRobot>& robots,
  const std::vector<MappedTask>& tasks,
  const OfflineMissionPlan& plan,
  const ExportOptions& options)
{
  return serialize_json(bundle, robots, tasks, plan, options);
}

void PlanExporter::write(
  const std::filesystem::path& output_directory,
  const MultiMapBundle& bundle,
  const std::vector<MappedRobot>& robots,
  const std::vector<MappedTask>& tasks,
  const OfflineMissionPlan& plan,
  const ExportOptions& options)
{
  std::filesystem::create_directories(output_directory);
  std::ofstream json_output(output_directory / "plan.json");
  if (!json_output)
    throw std::runtime_error("cannot write " + (output_directory / "plan.json").string());
  json_output << serialize_json(bundle, robots, tasks, plan, options);
  write_summary(output_directory / "summary.txt", bundle, robots, tasks, plan);

  std::vector<std::size_t> owner(tasks.size(), robots.size());
  for (const auto& route : plan.routes)
    for (const auto& stop : route.stops)
      for (const auto task : stop.task_indices)
        owner[task] = route.robot_index;

  for (const auto& [map_id, map] : bundle.maps) {
    auto image = cv::imread(map.image_path.string(), cv::IMREAD_COLOR);
    if (image.empty())
      throw std::runtime_error("cannot render map image: " + map.image_path.string());
    if (options.draw_grid) {
      for (int x = 0; x < image.cols; x += 20)
        cv::line(image, {x, 0}, {x, image.rows - 1}, {170, 170, 170}, 1);
      for (int y = 0; y < image.rows; y += 20)
        cv::line(image, {0, y}, {image.cols - 1, y}, {170, 170, 170}, 1);
    }

    for (std::size_t robot = 0; robot < robots.size(); ++robot) {
      std::vector<std::vector<cv::Point>> lines;
      std::vector<cv::Point> current;
      if (robot < plan.schedules.size()) {
        for (const auto& state : plan.schedules[robot]) {
          if (state.position.map_id == map_id) {
            const auto point = pixel(map, state.position);
            if (current.empty() || current.back() != point)
              current.push_back(point);
          } else if (!current.empty()) {
            lines.push_back(std::move(current));
            current.clear();
          }
        }
      } else {
        for (const auto& segment : plan.routes[robot].segments) {
          for (const auto& step : segment.steps) {
            if (step.position.map_id == map_id)
              current.push_back(pixel(map, step.position));
            else if (!current.empty()) {
              lines.push_back(std::move(current));
              current.clear();
            }
          }
        }
      }
      if (!current.empty()) lines.push_back(std::move(current));
      for (const auto& line : lines)
        if (line.size() > 1U)
          cv::polylines(image, line, false, colors()[robot % colors().size()],
            options.path_thickness, cv::LINE_AA);
    }

    for (std::size_t i = 0; i < tasks.size(); ++i) {
      if (tasks[i].location.map_id != map_id || owner[i] >= robots.size())
        continue;
      const auto point = pixel(map, tasks[i].location);
      cv::circle(image, point, 7, {255, 255, 255}, -1, cv::LINE_AA);
      cv::circle(image, point, 6, colors()[owner[i] % colors().size()], 2, cv::LINE_AA);
      cv::putText(image, std::to_string(i + 1U), point + cv::Point(8, -5),
        cv::FONT_HERSHEY_SIMPLEX, 0.42, {80, 80, 80}, 1, cv::LINE_AA);
    }
    for (std::size_t i = 0; i < robots.size(); ++i) {
      if (robots[i].start.map_id != map_id)
        continue;
      const auto point = pixel(map, robots[i].start);
      cv::circle(image, point, 9, {255, 255, 255}, -1, cv::LINE_AA);
      cv::circle(image, point, 8, colors()[i % colors().size()], -1, cv::LINE_AA);
      cv::putText(image, robots[i].id, point + cv::Point(-4, 4),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, {255, 255, 255}, 1, cv::LINE_AA);
    }
    for (const auto& transition : bundle.transitions) {
      if (transition.from_map != map_id)
        continue;
      const auto point = pixel(map, transition.from_cell);
      const std::array<cv::Point, 4> diamond{{
        point + cv::Point(0, -7), point + cv::Point(7, 0),
        point + cv::Point(0, 7), point + cv::Point(-7, 0)}};
      cv::polylines(image, diamond, true, {30, 30, 30}, 1, cv::LINE_AA);
    }
    const auto output_path = output_directory / ("routes_" + map_id + ".png");
    if (!cv::imwrite(output_path.string(), image))
      throw std::runtime_error("cannot write " + output_path.string());
  }
}

} // namespace capability_mission_planner::offline
