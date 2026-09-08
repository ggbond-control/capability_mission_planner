#include "multi_map_coordinator.hpp"

#include <capability_mission_planner/search/conflict_based_search.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <unordered_set>

namespace capability_mission_planner::offline {
namespace {

struct RouteFrame {
  GridPosition position;
  std::string resource;
};

struct State {
  int tick = 0;
  std::size_t frame = 0;
  GridPosition position;
  std::string resource;

  bool operator==(const State& other) const {
    return tick == other.tick && frame == other.frame &&
      position == other.position && resource == other.resource;
  }
};

struct StateHash {
  std::size_t operator()(const State& state) const noexcept {
    std::size_t value = std::hash<std::string>{}(state.position.map_id);
    value ^= std::hash<std::string>{}(state.resource) + 0x9e3779b9U +
      (value << 6U) + (value >> 2U);
    value ^= static_cast<std::size_t>(static_cast<unsigned int>(state.position.x)) * 73856093U;
    value ^= static_cast<std::size_t>(static_cast<unsigned int>(state.position.y)) * 19349663U;
    value ^= static_cast<std::size_t>(state.tick) * 83492791U;
    value ^= state.frame * 2654435761U;
    return value;
  }
};

enum class Action { Wait, Advance };

struct VertexConstraint {
  int tick = 0;
  GridPosition position;
  bool operator==(const VertexConstraint& other) const {
    return tick == other.tick && position == other.position;
  }
};
struct VertexHash {
  std::size_t operator()(const VertexConstraint& value) const noexcept {
    return StateHash{}(State{value.tick, 0U, value.position, {}});
  }
};

struct ResourceConstraint {
  int tick = 0;
  std::string resource;
  bool operator==(const ResourceConstraint& other) const {
    return tick == other.tick && resource == other.resource;
  }
};
struct ResourceHash {
  std::size_t operator()(const ResourceConstraint& value) const noexcept {
    return std::hash<std::string>{}(value.resource) ^
      (static_cast<std::size_t>(value.tick) * 2654435761U);
  }
};

struct EdgeConstraint {
  int tick = 0;
  GridPosition from;
  GridPosition to;
  bool operator==(const EdgeConstraint& other) const {
    return tick == other.tick && from == other.from && to == other.to;
  }
};
struct EdgeHash {
  std::size_t operator()(const EdgeConstraint& value) const noexcept {
    return StateHash{}(State{value.tick, 0U, value.from, {}}) ^
      (StateHash{}(State{0, 0U, value.to, {}}) << 1U);
  }
};

struct Constraints {
  std::unordered_set<VertexConstraint, VertexHash> vertex;
  std::unordered_set<ResourceConstraint, ResourceHash> resource;
  std::unordered_set<EdgeConstraint, EdgeHash> edge;

  void add(const Constraints& other) {
    vertex.insert(other.vertex.begin(), other.vertex.end());
    resource.insert(other.resource.begin(), other.resource.end());
    edge.insert(other.edge.begin(), other.edge.end());
  }
  bool overlap(const Constraints& other) const {
    for (const auto& item : other.vertex)
      if (vertex.count(item) != 0U) return true;
    for (const auto& item : other.resource)
      if (resource.count(item) != 0U) return true;
    for (const auto& item : other.edge)
      if (edge.count(item) != 0U) return true;
    return false;
  }
};

struct Conflict {
  enum class Type { Vertex, Edge, Resource } type = Type::Vertex;
  int tick = 0;
  std::size_t first = 0;
  std::size_t second = 0;
  GridPosition first_from;
  GridPosition first_to;
  GridPosition second_from;
  GridPosition second_to;
  std::string resource;
};

using Plan = search::PlanResult<State, Action, int>;

const State& state_at(const Plan& plan, std::size_t tick) {
  return tick < plan.states.size() ? plan.states[tick].first : plan.states.back().first;
}

std::string edge_resource(const GridPosition& first, const GridPosition& second) {
  const auto& low = second < first ? second : first;
  const auto& high = second < first ? first : second;
  return "edge:" + low.map_id + ':' + std::to_string(low.x) + ':' +
    std::to_string(low.y) + ':' + std::to_string(high.x) + ':' +
    std::to_string(high.y);
}

bool is_in_transit(const std::string& resource) {
  return resource.compare(0, 5U, "edge:") == 0 ||
    resource.compare(0, 11U, "transition:") == 0;
}

void append_segment(std::vector<RouteFrame>& frames, const MultiMapPath& segment) {
  if (segment.steps.empty()) return;
  if (frames.back().position != segment.steps.front().position)
    throw std::logic_error("route segment does not start at the current position");
  for (std::size_t i = 1; i < segment.steps.size(); ++i) {
    const auto& from = segment.steps[i - 1U];
    const auto& to = segment.steps[i];
    const int duration = to.arrival_tick - from.arrival_tick;
    if (duration <= 0)
      throw std::logic_error("route segment arrival ticks must increase");
    const std::string resource = to.transition_id.empty()
      ? edge_resource(from.position, to.position)
      : "transition:" + to.transition_id;
    for (int tick = 1; tick < duration; ++tick)
      frames.push_back({from.position, resource});
    frames.push_back({to.position, {}});
  }
}

std::vector<RouteFrame> make_frames(const MappedRobot& robot, const MappedRobotRoute& route,
  const TraversalOptions& options) {
  std::vector<RouteFrame> frames{{robot.start, {}}};
  std::size_t segment = 0;
  for (const auto& stop : route.stops) {
    if (segment >= route.segments.size())
      throw std::logic_error("missing path segment for route stop");
    append_segment(frames, route.segments[segment++]);
    for (int tick = 0; tick < stop.service_ticks; ++tick)
      frames.push_back({stop.location, {}});
  }
  if (segment < route.segments.size()) append_segment(frames, route.segments[segment++]);
  if (segment != route.segments.size())
    throw std::logic_error("unexpected extra path segments in route");
  for (std::size_t i = 0; i < frames.size(); ++i) {
    if (!frames[i].resource.empty()) continue;
    for (const auto& resource : options.shared_resources) {
      if (std::find(resource.cells.begin(), resource.cells.end(), frames[i].position) !=
        resource.cells.end()) {
        frames[i].resource = "region:" + resource.id;
        break;
      }
    }
  }
  for (const auto& resource : options.shared_resources) {
    const int buffer = std::max(0, static_cast<int>(std::ceil(
      resource.buffer_seconds / options.time_step_seconds)));
    const std::string id = "region:" + resource.id;
    for (int i = 0; i < static_cast<int>(frames.size()); ++i) {
      if (frames[i].resource != id) continue;
      for (int offset = 1; offset <= buffer; ++offset) {
        if (i - offset > 0 && frames[i - offset].resource.empty()) frames[i - offset].resource = id;
        if (i + offset < static_cast<int>(frames.size()) && frames[i + offset].resource.empty()) frames[i + offset].resource = id;
      }
    }
  }
  if (options.resource_buffer_seconds > 0.0) {
    const int buffer = static_cast<int>(std::ceil(
      options.resource_buffer_seconds / options.time_step_seconds));
    for (std::size_t i = 0; i < frames.size(); ++i) {
      if (frames[i].resource.empty()) continue;
      const auto resource = frames[i].resource;
      for (int offset = 1; offset <= buffer; ++offset) {
        if (i > static_cast<std::size_t>(offset) && frames[i - offset].resource.empty()) frames[i - offset].resource = resource;
        if (i + static_cast<std::size_t>(offset) < frames.size() && frames[i + offset].resource.empty()) frames[i + offset].resource = resource;
      }
    }
  }
  if (!frames.empty() && !frames.back().resource.empty())
    frames.push_back({frames.back().position, {}});
  return frames;
}

class Environment {
public:
  Environment(std::vector<std::vector<RouteFrame>> frames,
    const MultiMapBundle& bundle, const std::vector<MappedRobot>& robots,
    CoordinationStats* stats)
  : _frames(std::move(frames)), _bundle(bundle), _robots(robots), _stats(stats) {}

  void set_cbs_phase(bool value) { _cbs_phase = value; }

  void setLowLevelContext(std::size_t agent, const Constraints* constraints) {
    _agent = agent;
    _constraints = constraints;
    _last_goal_constraint = -1;
    const auto& goal = _frames[agent].back().position;
    for (const auto& constraint : constraints->vertex)
      if (constraint.position == goal)
        _last_goal_constraint = std::max(_last_goal_constraint, constraint.tick);
    if (_stats) {
      if (_cbs_phase) ++_stats->cbs_low_level_searches;
      else ++_stats->prioritized_low_level_searches;
    }
  }

  int admissibleHeuristic(const State& state) const {
    return static_cast<int>(_frames[_agent].size() - 1U - state.frame);
  }
  bool isSolution(const State& state) const {
    return state.frame + 1U == _frames[_agent].size() && state.tick > _last_goal_constraint;
  }

  void getNeighbors(const State& state,
    std::vector<search::Neighbor<State, Action, int>>& neighbors) const
  {
    // A robot may wait at a cell, but not midway through an edge or transition.
    if (!is_in_transit(state.resource)) {
      State waiting = state;
      ++waiting.tick;
      if (allowed(state, waiting)) neighbors.emplace_back(std::move(waiting), Action::Wait, 1);
    }
    if (state.frame + 1U < _frames[_agent].size()) {
      const auto& frame = _frames[_agent][state.frame + 1U];
      State next{state.tick + 1, state.frame + 1U, frame.position, frame.resource};
      if (allowed(state, next)) neighbors.emplace_back(std::move(next), Action::Advance, 1);
    }
  }

  bool getFirstConflict(const std::vector<Plan>& plans, Conflict& output) const {
    const auto started = std::chrono::steady_clock::now();
    const auto finish = [&]() {
      if (!_stats) return;
      ++_stats->conflict_checks;
      _stats->conflict_check_seconds += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    };
    std::size_t horizon = 0;
    for (const auto& plan : plans) horizon = std::max(horizon, plan.states.size());
    for (std::size_t tick = 0; tick < horizon; ++tick) {
      for (std::size_t a = 0; a < plans.size(); ++a) {
        for (std::size_t b = a + 1U; b < plans.size(); ++b) {
          const auto& first = state_at(plans[a], tick);
          const auto& second = state_at(plans[b], tick);
          if (!first.resource.empty() && first.resource == second.resource) {
            output = Conflict{Conflict::Type::Resource, static_cast<int>(tick),
              a, b, {}, {}, {}, {}, first.resource};
            finish();
            return true;
          }
          if (positions_too_close(first.position, second.position, a, b)) {
            output = Conflict{Conflict::Type::Vertex, static_cast<int>(tick),
              a, b, first.position, first.position, second.position,
              second.position, {}};
            finish();
            return true;
          }
          if (tick + 1U >= horizon) continue;
          const auto& first_next = state_at(plans[a], tick + 1U);
          const auto& second_next = state_at(plans[b], tick + 1U);
          if (first.resource.empty() && second.resource.empty() &&
            first_next.resource.empty() && second_next.resource.empty() &&
            swept_paths_too_close(first.position, first_next.position,
              second.position, second_next.position, a, b))
          {
            output = Conflict{Conflict::Type::Edge, static_cast<int>(tick),
              a, b, first.position, first_next.position,
              second.position, second_next.position, {}};
            finish();
            return true;
          }
        }
      }
    }
    finish();
    return false;
  }

  void createConstraintsFromConflict(const Conflict& conflict,
    std::map<std::size_t, Constraints>& output) const
  {
    if (conflict.type == Conflict::Type::Vertex) {
      Constraints first;
      first.vertex.insert({conflict.tick, conflict.first_to});
      output[conflict.first] = first;
      Constraints second;
      second.vertex.insert({conflict.tick, conflict.second_to});
      output[conflict.second] = second;
    } else if (conflict.type == Conflict::Type::Resource) {
      Constraints constraint;
      constraint.resource.insert({conflict.tick, conflict.resource});
      output[conflict.first] = constraint;
      output[conflict.second] = constraint;
    } else {
      Constraints first;
      first.edge.insert({conflict.tick, conflict.first_from, conflict.first_to});
      output[conflict.first] = first;
      Constraints second;
      second.edge.insert({conflict.tick, conflict.second_from, conflict.second_to});
      output[conflict.second] = second;
    }
  }

  void onExpandHighLevelNode(int) {
    if (_stats && _cbs_phase) ++_stats->cbs_high_level_expanded_nodes;
  }
  void onExpandLowLevelNode(const State&, int, int) {
    if (!_stats) return;
    if (_cbs_phase) ++_stats->cbs_low_level_expanded_nodes;
    else ++_stats->prioritized_low_level_expanded_nodes;
  }

  bool allowedAgainstPlans(std::size_t agent, const State& from, const State& to,
    const std::vector<Plan>& plans, const std::vector<std::size_t>& fixed) const {
    for (const auto other : fixed) {
      const auto& other_to = state_at(plans[other], static_cast<std::size_t>(to.tick));
      if (!to.resource.empty() && to.resource == other_to.resource) return false;
      if (positions_too_close(to.position, other_to.position, agent, other)) return false;
      const auto& other_from = state_at(plans[other], static_cast<std::size_t>(from.tick));
      if (from.resource.empty() && to.resource.empty() &&
        other_from.resource.empty() && other_to.resource.empty() &&
        swept_paths_too_close(from.position, to.position, other_from.position,
          other_to.position, agent, other)) return false;
    }
    return true;
  }



private:
  MetricPose root_pose(const GridPosition& position) const {
    const auto local = _bundle.map(position.map_id).grid_to_local(position);
    return _bundle.map(position.map_id).local_to_root(local);
  }

  double safety_distance(std::size_t first, std::size_t second) const {
    return _robots[first].footprint_radius_m + _robots[second].footprint_radius_m +
      _robots[first].safety_margin_m + _robots[second].safety_margin_m;
  }

  bool positions_too_close(const GridPosition& first, const GridPosition& second,
    std::size_t a, std::size_t b) const {
    if (first.map_id != second.map_id) return false;
    const auto p = root_pose(first); const auto q = root_pose(second);
    return std::hypot(p.x - q.x, p.y - q.y) <= safety_distance(a, b) + 1e-9;
  }

  bool swept_paths_too_close(const GridPosition& af, const GridPosition& at,
    const GridPosition& bf, const GridPosition& bt, std::size_t a, std::size_t b) const {
    if (af.map_id != at.map_id || bf.map_id != bt.map_id || af.map_id != bf.map_id) return false;
    const auto p = root_pose(af), q = root_pose(at), r = root_pose(bf), s = root_pose(bt);
    const double dx = p.x - r.x, dy = p.y - r.y;
    const double vx = (q.x - p.x) - (s.x - r.x);
    const double vy = (q.y - p.y) - (s.y - r.y);
    const double vv = vx * vx + vy * vy;
    const double t = vv > 1e-12 ? std::clamp(-(dx * vx + dy * vy) / vv, 0.0, 1.0) : 0.0;
    return std::hypot(dx + t * vx, dy + t * vy) <= safety_distance(a, b) + 1e-9;
  }

  bool allowed(const State& from, const State& to) const {
    if (_constraints->vertex.count({to.tick, to.position}) != 0U) return false;
    if (!to.resource.empty() &&
      _constraints->resource.count({to.tick, to.resource}) != 0U) return false;
    if (from.resource.empty() && to.resource.empty() &&
      _constraints->edge.count({from.tick, from.position, to.position}) != 0U) return false;
    return true;
  }

  std::vector<std::vector<RouteFrame>> _frames;
  const MultiMapBundle& _bundle;
  const std::vector<MappedRobot>& _robots;
  CoordinationStats* _stats = nullptr;
  bool _cbs_phase = false;
  std::size_t _agent = 0;
  const Constraints* _constraints = nullptr;
  int _last_goal_constraint = -1;
};

class PrioritizedEnvironment {
public:
  PrioritizedEnvironment(Environment& environment, std::size_t agent,
    const std::vector<Plan>& plans, const std::vector<std::size_t>& fixed,
    int maximum_tick, double heuristic_weight)
  : _environment(environment), _agent(agent), _plans(plans), _fixed(fixed),
    _maximum_tick(maximum_tick), _heuristic_weight(heuristic_weight) {
    _environment.setLowLevelContext(agent, &_constraints);
  }
  int admissibleHeuristic(const State& state) const {
    return static_cast<int>(std::ceil(
      _environment.admissibleHeuristic(state) * _heuristic_weight));
  }
  bool isSolution(const State& state) const { return _environment.isSolution(state); }
  void getNeighbors(const State& state,
    std::vector<search::Neighbor<State, Action, int>>& neighbors) const {
    if (state.tick >= _maximum_tick) { neighbors.clear(); return; }
    _environment.getNeighbors(state, neighbors);
    neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(), [&](const auto& next) {
      return !_environment.allowedAgainstPlans(
        _agent, state, next.state, _plans, _fixed);
    }), neighbors.end());
  }
  void onExpandNode(const State& state, int f_score, int g_score) const {
    _environment.onExpandLowLevelNode(state, f_score, g_score);
  }
  void onDiscover(const State&, int, int) const {}

private:
  Environment& _environment;
  std::size_t _agent;
  const std::vector<Plan>& _plans;
  const std::vector<std::size_t>& _fixed;
  int _maximum_tick;
  double _heuristic_weight;
  Constraints _constraints;
};

bool prioritized_schedule(Environment& environment, const std::vector<State>& starts,
  int maximum_tick, std::vector<Plan>& plans) {
  constexpr double heuristic_weight = 2.0;
  plans.assign(starts.size(), {});
  std::vector<std::size_t> fixed;
  for (std::size_t agent = 0; agent < starts.size(); ++agent) {
    PrioritizedEnvironment low_level_environment(
      environment, agent, plans, fixed, maximum_tick, heuristic_weight);
    search::AStar<State, Action, int, PrioritizedEnvironment, StateHash> low_level(
      low_level_environment);
    if (!low_level.search(starts[agent], plans[agent])) return false;
    fixed.push_back(agent);
  }
  Conflict conflict;
  return !environment.getFirstConflict(plans, conflict);
}

} // namespace

std::vector<std::vector<TimedMapState>> coordinate_multi_map_routes(
  const MultiMapPathPlanner& path_planner,
  const std::vector<MappedRobot>& robots,
  const std::vector<MappedRobotRoute>& routes,
  CoordinationStats* stats)
{
  if (robots.size() != routes.size())
    throw std::invalid_argument("robot and route counts must match");
  std::vector<std::vector<RouteFrame>> frames;
  std::vector<State> starts;
  std::size_t maximum_tick = 0U;
  frames.reserve(robots.size());
  starts.reserve(robots.size());
  for (std::size_t i = 0; i < robots.size(); ++i) {
    frames.push_back(make_frames(robots[i], routes[i], path_planner.options()));
    maximum_tick += frames.back().size();
    starts.push_back(State{0, 0U, frames.back().front().position,
      frames.back().front().resource});
  }

  if (stats) *stats = {};
  Environment environment(std::move(frames), path_planner.bundle(), robots, stats);
  std::vector<Plan> plans;
  if (stats) stats->prioritized_attempted = true;
  const bool prioritized_succeeded = prioritized_schedule(environment, starts,
    static_cast<int>(std::min(maximum_tick,
      static_cast<std::size_t>(std::numeric_limits<int>::max()))), plans);
  if (stats) stats->prioritized_succeeded = prioritized_succeeded;
  if (!prioritized_succeeded) {
    if (stats) stats->cbs_started = true;
    environment.set_cbs_phase(true);
    search::CBS<State, Action, int, Conflict, Constraints, Environment, StateHash> cbs(
      environment, path_planner.options().coordination_max_high_level_nodes);
    if (!cbs.search(starts, plans)) {
      if (cbs.limit_reached())
        throw std::runtime_error("multi-map CBS reached its high-level node limit");
      throw std::runtime_error("multi-map CBS could not find a conflict-free schedule");
    }
    if (stats) stats->cbs_succeeded = true;
  }

  std::vector<std::vector<TimedMapState>> output(plans.size());
  for (std::size_t i = 0; i < plans.size(); ++i) {
    output[i].reserve(plans[i].states.size());
    for (const auto& [state, cost] : plans[i].states) {
      (void)cost;
      std::string transition;
      constexpr const char prefix[] = "transition:";
      if (state.resource.compare(0, sizeof(prefix) - 1U, prefix) == 0)
        transition = state.resource.substr(sizeof(prefix) - 1U);
      output[i].push_back({state.position, state.tick, std::move(transition),
        state.frame, state.resource});
    }
    for (std::size_t frame = 1; frame < plans[i].states.size(); ++frame) {
      const auto& previous = plans[i].states[frame - 1U].first;
      const auto& current = plans[i].states[frame].first;
      if (current.frame == previous.frame && stats)
        ++stats->total_wait_ticks;
    }
  }
  return output;
}

} // namespace capability_mission_planner::offline
