#include <capability_mission_planner/offline_map_planner.hpp>

#include "multi_map_coordinator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <utility>

namespace capability_mission_planner::offline
{
    namespace
    {
        bool compatible(const MappedRobot &robot, const MappedTask &task)
        {
            return std::all_of(task.requirements.begin(), task.requirements.end(),
                               [&](const auto &requirement)
                               {
                                   return robot.capabilities.count(requirement) != 0U;
                               });
        }

        void recompute_route(
            MappedRobotRoute &route,
            const MappedRobot &robot,
            const MultiMapPathPlanner &planner,
            bool exact_paths)
        {
            route.travel_ticks = 0;
            route.service_ticks = 0;
            route.segments.clear();
            GridPosition current = robot.start;
            const double clearance = std::max(0.0, robot.clearance_radius_m + robot.safety_margin_m);
            const double speed = robot.nominal_speed_mps > 0.0 ? robot.nominal_speed_mps : planner.options().nominal_speed_mps;
            for (const auto &stop : route.stops)
            {
                const auto path = exact_paths ? planner.plan_exact(current, stop.location, robot.capabilities, clearance, speed) : planner.plan(current, stop.location, robot.capabilities, clearance, speed);
                route.travel_ticks += path.travel_ticks;
                route.service_ticks += stop.service_ticks;
                route.segments.push_back(path);
                current = stop.location;
            }
            if (robot.return_position && !route.stops.empty())
            {
                const auto path = exact_paths ? planner.plan_exact(current, *robot.return_position, robot.capabilities, clearance, speed) : planner.plan(current, *robot.return_position, robot.capabilities, clearance, speed);
                route.travel_ticks += path.travel_ticks;
                route.segments.push_back(path);
            }
        }

        double objective(
            const std::vector<MappedRobotRoute> &routes,
            const ObjectiveWeights &weights)
        {
            int maximum = 0;
            int total = 0;
            for (const auto &route : routes)
            {
                maximum = std::max(maximum, route.load_ticks());
                total += route.load_ticks();
            }
            return weights.maximum_load * maximum + weights.total_load * total;
        }

        double objective_with_replacement(
            const std::vector<MappedRobotRoute> &routes,
            std::size_t replaced,
            const MappedRobotRoute &candidate,
            const ObjectiveWeights &weights)
        {
            int maximum = 0;
            int total = 0;
            for (std::size_t i = 0; i < routes.size(); ++i)
            {
                const int load = i == replaced ? candidate.load_ticks() : routes[i].load_ticks();
                maximum = std::max(maximum, load);
                total += load;
            }
            return weights.maximum_load * maximum + weights.total_load * total;
        }

        // Allocation only needs edge travel estimates. Keep those estimates with the
        // route so a local insertion/removal updates at most two adjacent edges.
        struct RouteState
        {
            MappedRobotRoute route;
            std::vector<int> edges;
            const MappedRobot *robot = nullptr;
            const MultiMapPathPlanner *planner = nullptr;

            int edge(const GridPosition &from, const GridPosition &to) const
            {
                const double clearance = std::max(0.0, robot->clearance_radius_m + robot->safety_margin_m);
                const double speed = robot->nominal_speed_mps > 0.0 ? robot->nominal_speed_mps : planner->options().nominal_speed_mps;
                return planner->estimate_distance(from, to, robot->capabilities, clearance, speed);
            }

            void rebuild()
            {
                edges.clear();
                GridPosition current = robot->start;
                route.travel_ticks = 0;
                route.service_ticks = 0;
                for (const auto &stop : route.stops)
                {
                    const int value = edge(current, stop.location);
                    edges.push_back(value);
                    route.travel_ticks += value;
                    route.service_ticks += stop.service_ticks;
                    current = stop.location;
                }
                if (robot->return_position && !route.stops.empty())
                {
                    const int value = edge(current, *robot->return_position);
                    edges.push_back(value);
                    route.travel_ticks += value;
                }
            }

            RouteState inserted(std::size_t position, const MappedRouteStop &stop) const
            {
                RouteState result = *this;
                result.route.stops.insert(result.route.stops.begin() + static_cast<std::ptrdiff_t>(position), stop);
                const GridPosition prev = position == 0U ? robot->start : route.stops[position - 1U].location;
                const GridPosition next = position == route.stops.size() ? (robot->return_position ? *robot->return_position : GridPosition{}) : route.stops[position].location;
                const int old_edge = position < edges.size() ? edges[position] : 0;
                const int first = result.edge(prev, stop.location);
                const int second = (position == route.stops.size() && !robot->return_position) ? 0 : result.edge(stop.location, next);
                result.route.travel_ticks += first + second - old_edge;
                result.route.service_ticks += stop.service_ticks;
                result.edges.insert(result.edges.begin() + static_cast<std::ptrdiff_t>(position), first);
                if (position < edges.size())
                    result.edges[position + 1U] = second;
                else if (robot->return_position)
                    result.edges.push_back(second);
                return result;
            }

            RouteState merged(std::size_t position, std::size_t task_index, int service_ticks,
                              double tolerance) const
            {
                RouteState result = *this;
                auto &stop = result.route.stops[position];
                stop.task_indices.push_back(task_index);
                stop.service_ticks += service_ticks;
                stop.position_tolerance_m = std::max(stop.position_tolerance_m, tolerance);
                result.route.service_ticks += service_ticks;
                return result;
            }
        };

        std::vector<RouteState> state_insertion_options(const RouteState &source,
                                                        std::size_t task_index, const MappedTask &task, int service_ticks)
        {
            for (std::size_t i = 0; i < source.route.stops.size(); ++i)
                if (source.route.stops[i].location == task.location)
                    return {source.merged(i, task_index, service_ticks, task.position_tolerance_m)};
            std::vector<RouteState> result;
            std::vector<std::size_t> positions;
            for (std::size_t i = 0; i <= source.route.stops.size(); ++i)
                positions.push_back(i);
            for (const auto i : positions)
                result.push_back(source.inserted(i, MappedRouteStop{task.location, {task_index}, service_ticks, task.position_tolerance_m}));
            return result;
        }

        std::size_t compatible_robot_count(
            const std::vector<MappedRobot> &robots,
            const MappedTask &task)
        {
            return static_cast<std::size_t>(std::count_if(
                robots.begin(), robots.end(), [&](const auto &robot)
                { return compatible(robot, task); }));
        }

        const SharedResource *resource_at(const std::vector<SharedResource> &resources,
                                          const GridPosition &position)
        {
            for (const auto &resource : resources)
                if (std::find(resource.cells.begin(), resource.cells.end(), position) != resource.cells.end())
                    return &resource;
            return nullptr;
        }

        int tick_at_frame(const std::vector<TimedMapState> &schedule, std::size_t frame)
        {
            const auto found = std::find_if(schedule.begin(), schedule.end(), [&](const auto &state)
                                            { return state.route_frame == frame; });
            return found == schedule.end() ? schedule.back().tick : found->tick;
        }

        void append_path_checkpoints(std::vector<NavigationCheckpoint> &points,
                                     const MultiMapPath &path, std::size_t frame_offset,
                                     const std::vector<TimedMapState> &schedule)
        {
            struct TurnCandidate
            {
                std::size_t index;
                GridPosition position;
                int tick;
            };
            std::vector<TurnCandidate> candidates;
            for (std::size_t i = 1; i + 1U < path.steps.size(); ++i)
            {
                const auto &previous = path.steps[i - 1U].position;
                const auto &current = path.steps[i].position;
                const auto &next = path.steps[i + 1U].position;
                if (previous.map_id != current.map_id || current.map_id != next.map_id)
                    continue;
                const long long dx1 = current.x - previous.x, dy1 = current.y - previous.y;
                const long long dx2 = next.x - current.x, dy2 = next.y - current.y;
                const double norm1 = std::hypot(static_cast<double>(dx1), static_cast<double>(dy1));
                const double norm2 = std::hypot(static_cast<double>(dx2), static_cast<double>(dy2));
                const double cosine = (dx1 * dx2 + dy1 * dy2) / (norm1 * norm2);
                if (cosine > std::cos(45.0 * 3.14159265358979323846 / 180.0))
                    continue;
                const auto frame = frame_offset + static_cast<std::size_t>(path.steps[i].arrival_tick);
                const int tick = tick_at_frame(schedule, frame);
                candidates.push_back({i, current, tick});
            }
            // A grid path often represents one physical bend with several alternating
            // diagonal/cardinal steps. Collapse candidates that are close along the
            // same bend and retain its middle point as the navigation waypoint.
            constexpr std::size_t max_turn_run = 8U;
            std::size_t begin = 0;
            while (begin < candidates.size())
            {
                std::size_t end = begin;
                while (end + 1U < candidates.size() &&
                       candidates[end + 1U].index - candidates[end].index <= max_turn_run)
                    ++end;
                const auto &representative = candidates[begin + (end - begin) / 2U];
                points.push_back({NavigationCheckpointType::Turn, representative.position, representative.tick, representative.tick, {}, {}, {}, {}});
                begin = end + 1U;
            }
            for (std::size_t i = 1; i < path.steps.size(); ++i)
            {
                if (path.steps[i].transition_id.empty())
                    continue;
                const auto &id = path.steps[i].transition_id;
                const auto entry_frame = frame_offset +
                                         static_cast<std::size_t>(path.steps[i - 1U].arrival_tick);
                const auto exit_frame = frame_offset +
                                        static_cast<std::size_t>(path.steps[i].arrival_tick);
                const int entry_tick = tick_at_frame(schedule, entry_frame);
                const int exit_tick = tick_at_frame(schedule, exit_frame);
                points.push_back({NavigationCheckpointType::TransitionEntry,
                                  path.steps[i - 1U].position,
                                  entry_tick,
                                  entry_tick,
                                  {},
                                  id,
                                  {},
                                  {}});
                points.push_back({NavigationCheckpointType::TransitionExit,
                                  path.steps[i].position,
                                  exit_tick,
                                  exit_tick,
                                  {},
                                  id,
                                  {},
                                  {}});
            }
        }

        void extract_navigation_annotations(OfflineMissionPlan &plan,
                                            const std::vector<MappedTask> &tasks)
        {
            plan.navigation_checkpoints.assign(plan.schedules.size(), {});
            plan.traffic_events.assign(plan.schedules.size(), {});
            for (std::size_t robot = 0; robot < plan.schedules.size(); ++robot)
            {
                const auto &schedule = plan.schedules[robot];
                if (schedule.empty())
                    continue;
                auto &points = plan.navigation_checkpoints[robot];
                points.push_back({NavigationCheckpointType::Start, schedule.front().position, schedule.front().tick, schedule.front().tick, {}, {}, {}, {}});
                const auto &route = plan.routes.at(robot);
                std::size_t frame = 0;
                std::size_t segment = 0;
                for (const auto &stop : route.stops)
                {
                    const auto &path = route.segments.at(segment++);
                    append_path_checkpoints(points, path, frame, schedule);
                    frame += static_cast<std::size_t>(path.travel_ticks);
                    const int arrival_tick = tick_at_frame(schedule, frame);
                    const int departure_tick = tick_at_frame(schedule,
                                                             frame + static_cast<std::size_t>(stop.service_ticks));
                    for (const auto task_index : stop.task_indices)
                    {
                        const auto task_id = tasks.at(task_index).id();
                        points.push_back({NavigationCheckpointType::Task, stop.location, arrival_tick, departure_tick, {}, {}, task_id, {}});
                    }
                    frame += static_cast<std::size_t>(stop.service_ticks);
                }
                if (segment < route.segments.size())
                {
                    append_path_checkpoints(points, route.segments.at(segment), frame, schedule);
                }

                std::size_t wait_index = 0;
                for (std::size_t i = 0; i < schedule.size(); ++i)
                {
                    const auto &state = schedule[i];
                    const auto *before = resource_at(plan.shared_resources, state.position);
                    const auto *after = i + 1U < schedule.size()
                                            ? resource_at(plan.shared_resources, schedule[i + 1U].position)
                                            : nullptr;
                    if (before != after)
                    {
                        if (before)
                            points.push_back({NavigationCheckpointType::ResourceExit, state.position, state.tick, state.tick, before->id, {}, {}, {}});
                        if (after)
                            points.push_back({NavigationCheckpointType::ResourceEntry,
                                              schedule[i + 1U].position,
                                              schedule[i + 1U].tick,
                                              schedule[i + 1U].tick,
                                              after->id,
                                              {},
                                              {},
                                              {}});
                    }
                    std::size_t end = i;
                    while (end + 1U < schedule.size() &&
                           schedule[end + 1U].route_frame == state.route_frame)
                        ++end;
                    if (end > i)
                    {
                        const std::string checkpoint_id = "holding-" + std::to_string(wait_index++);
                        points.push_back({NavigationCheckpointType::Holding, state.position, state.tick, schedule[end].tick, {}, {}, {}, checkpoint_id});
                        plan.traffic_events[robot].push_back({"wait", state.tick, schedule[end].tick, state.position, {}, "coordination_delay", checkpoint_id});
                        i = end;
                    }
                }
                points.push_back({NavigationCheckpointType::Finish, schedule.back().position, schedule.back().tick, schedule.back().tick, {}, {}, {}, {}});
                std::stable_sort(points.begin(), points.end(), [](const auto &a, const auto &b)
                                 { return a.arrival_tick < b.arrival_tick; });
                points.erase(std::unique(points.begin(), points.end(), [](const auto &a, const auto &b)
                                         { return a.type == NavigationCheckpointType::Turn &&
                                                  b.type == NavigationCheckpointType::Turn; }),
                             points.end());
                for (std::size_t i = 0; i < points.size(); ++i)
                    if (points[i].id.empty())
                        points[i].id = "checkpoint-" + std::to_string(i);
            }
        }

        std::vector<MappedRobotRoute> build_routes(
            const std::vector<MappedRobot> &robots, const std::vector<MappedTask> &tasks,
            const std::vector<int> &service_ticks, const MultiMapPathPlanner &planner,
            const ObjectiveWeights &weights, unsigned int seed)
        {
            std::vector<MappedRobotRoute> routes(robots.size());
            for (std::size_t i = 0; i < routes.size(); ++i)
                routes[i].robot_index = i;
            std::vector<RouteState> states(robots.size());
            for (std::size_t i = 0; i < states.size(); ++i)
            {
                states[i].route = routes[i];
                states[i].robot = &robots[i];
                states[i].planner = &planner;
                states[i].rebuild();
            }
            std::set<std::size_t> remaining;
            for (std::size_t i = 0; i < tasks.size(); ++i)
                remaining.insert(i);
            (void)seed;
            while (!remaining.empty())
            {
                std::vector<std::size_t> shortlist(remaining.begin(), remaining.end());
                std::stable_sort(shortlist.begin(), shortlist.end(), [&](std::size_t a, std::size_t b)
                                 {
      const auto ca = compatible_robot_count(robots, tasks[a]);
      const auto cb = compatible_robot_count(robots, tasks[b]);
      if (ca != cb) return ca < cb;
      if (tasks[a].high_priority() != tasks[b].high_priority()) return tasks[a].high_priority();
      return service_ticks[a] > service_ticks[b]; });
                const std::size_t regret_candidate_limit = tasks.size() > 50U ? 1U : 12U;
                if (shortlist.size() > regret_candidate_limit)
                    shortlist.resize(regret_candidate_limit);
                struct Choice
                {
                    std::size_t task;
                    double regret;
                };
                std::vector<Choice> choices;
                for (const auto ti : shortlist)
                {
                    double first = std::numeric_limits<double>::infinity(), second = first;
                    for (std::size_t r = 0; r < robots.size(); ++r)
                        if (compatible(robots[r], tasks[ti]))
                        {
                            for (const auto &candidate : state_insertion_options(states[r], ti, tasks[ti], service_ticks[ti]))
                            {
                                const double score = objective_with_replacement(
                                    routes, r, candidate.route, weights);
                                if (score < first)
                                {
                                    second = first;
                                    first = score;
                                }
                                else if (score < second)
                                    second = score;
                            }
                        }
                    if (std::isfinite(first))
                        choices.push_back({ti, std::isfinite(second) ? second - first : 1e9});
                }
                if (choices.empty())
                    throw std::runtime_error("no reachable capable robot for remaining task");
                std::stable_sort(choices.begin(), choices.end(), [](const auto &a, const auto &b)
                                 { return a.regret > b.regret; });
                const auto ti = choices.front().task;
                double best = std::numeric_limits<double>::infinity();
                std::size_t owner = robots.size();
                MappedRobotRoute selected;
                for (std::size_t r = 0; r < robots.size(); ++r)
                    if (compatible(robots[r], tasks[ti]))
                    {
                        for (auto candidate : state_insertion_options(states[r], ti, tasks[ti], service_ticks[ti]))
                        {
                            const double score = objective_with_replacement(
                                routes, r, candidate.route, weights);
                            if (score < best)
                            {
                                best = score;
                                owner = r;
                                selected = std::move(candidate.route);
                            }
                        }
                    }
                routes[owner] = std::move(selected);
                states[owner].route = routes[owner];
                states[owner].rebuild();
                remaining.erase(ti);
            }

            // Relocation/swap neighborhoods are useful on small instances, but their
            // full-route recomputation grows too quickly for dense task sets.
            const auto max_passes = tasks.size() <= 20U
                                        ? std::min<std::size_t>(tasks.size() + 1U, 8U)
                                        : 0U;
            for (std::size_t pass = 0; pass < max_passes; ++pass)
            {
                std::vector<RouteState> pass_states(robots.size());
                for (std::size_t i = 0; i < robots.size(); ++i)
                {
                    pass_states[i].route = routes[i];
                    pass_states[i].robot = &robots[i];
                    pass_states[i].planner = &planner;
                    pass_states[i].rebuild();
                }
                double best = objective(routes, weights);
                std::vector<MappedRobotRoute> improved;
                for (std::size_t source = 0; source < robots.size(); ++source)
                {
                    std::vector<std::size_t> assigned;
                    for (const auto &stop : routes[source].stops)
                        assigned.insert(assigned.end(), stop.task_indices.begin(), stop.task_indices.end());
                    for (const auto ti : assigned)
                    {
                        auto without_state = pass_states[source];
                        for (std::size_t i = 0; i < without_state.route.stops.size(); ++i)
                            if (std::find(without_state.route.stops[i].task_indices.begin(), without_state.route.stops[i].task_indices.end(), ti) != without_state.route.stops[i].task_indices.end())
                            {
                                without_state.route.stops[i].task_indices.erase(std::find(without_state.route.stops[i].task_indices.begin(), without_state.route.stops[i].task_indices.end(), ti));
                                without_state.route.stops[i].service_ticks -= service_ticks[ti];
                                if (without_state.route.stops[i].task_indices.empty())
                                    without_state.route.stops.erase(without_state.route.stops.begin() + static_cast<std::ptrdiff_t>(i));
                                without_state.rebuild();
                                break;
                            }
                        auto without = routes;
                        without[source] = without_state.route;
                        for (std::size_t target = 0; target < robots.size(); ++target)
                            if (compatible(robots[target], tasks[ti]))
                                for (const auto &candidate : state_insertion_options(target == source ? without_state : pass_states[target], ti, tasks[ti], service_ticks[ti]))
                                {
                                    auto trial = without;
                                    trial[target] = candidate.route;
                                    const double score = objective(trial, weights);
                                    if (score + 1e-9 < best)
                                    {
                                        best = score;
                                        improved = std::move(trial);
                                    }
                                }
                    }
                }
                if (improved.empty())
                    break;
                routes = std::move(improved);
            }

            return routes;
        }
    } // namespace

    int MappedTask::service_duration_seconds() const
    {
        const auto seconds = std::chrono::duration<double>(
                                 header.original_duration_estimate())
                                 .count();
        return std::max(0, static_cast<int>(std::ceil(seconds)));
    }

    bool MappedTask::high_priority() const
    {
        return booking->priority() == TaskPriority::High;
    }

    MappedTask make_mapped_task(
        std::string id,
        GridPosition location,
        CapabilitySet requirements,
        std::string category,
        int service_seconds,
        bool high_priority,
        int earliest_start_seconds)
    {
        const auto priority = high_priority ? TaskPriority::High : TaskPriority::Normal;
        const auto start = TaskTime{} + std::chrono::seconds(earliest_start_seconds);
        auto booking = std::make_shared<TaskBooking>(
            std::move(id), start, priority, false, std::vector<std::string>{category});
        TaskHeader header(
            std::move(category), "offline mapped atomic task",
            std::chrono::seconds(service_seconds));
        return {std::move(booking), std::move(header), std::move(location),
                std::move(requirements), 0.0};
    }

    OfflineMissionPlanner::OfflineMissionPlanner(
        MultiMapPathPlanner path_planner,
        ObjectiveWeights weights)
        : _path_planner(std::move(path_planner)), _weights(weights)
    {
        if (weights.maximum_load < 0.0 || weights.total_load < 0.0)
            throw std::invalid_argument("objective weights must be non-negative");
    }

    OfflineMissionPlan OfflineMissionPlanner::plan(
        const std::vector<MappedRobot> &robots,
        const std::vector<MappedTask> &tasks,
        bool coordinate_conflicts) const
    {
        if (robots.empty() && !tasks.empty())
            throw std::invalid_argument("tasks cannot be assigned without robots");
        std::set<std::string> robot_ids;
        std::set<GridPosition> starts;
        for (const auto &robot : robots)
        {
            if (!robot_ids.insert(robot.id).second)
                throw std::invalid_argument("duplicate robot id: " + robot.id);
            if (!_path_planner.bundle().traversable(robot.start))
                throw std::invalid_argument("robot start is not traversable: " + robot.id);
            if (robot.return_position &&
                !_path_planner.bundle().traversable(*robot.return_position))
                throw std::invalid_argument("robot return_home is not traversable: " + robot.id);
            if (!starts.insert(robot.start).second)
                throw std::invalid_argument("robots cannot share an initial location");
        }
        std::set<std::string> task_ids;
        for (const auto &task : tasks)
        {
            if (!task_ids.insert(task.id()).second)
                throw std::invalid_argument("duplicate task id: " + task.id());
            if (!_path_planner.bundle().traversable(task.location))
                throw std::invalid_argument("task location is not traversable: " + task.id());
            if (compatible_robot_count(robots, task) == 0U)
                throw std::invalid_argument("no capable robot for task " + task.id());
        }

        const double time_step = _path_planner.options().time_step_seconds;
        std::vector<int> service_ticks(tasks.size());
        std::transform(tasks.begin(), tasks.end(), service_ticks.begin(), [&](const auto &task)
                       { return static_cast<int>(std::ceil(
                             task.service_duration_seconds() / time_step - 1e-9)); });

        _path_planner.reset_stats();
        const auto allocation_started = std::chrono::steady_clock::now();
        std::vector<GridPosition> estimate_positions;
        estimate_positions.reserve(robots.size() + tasks.size());
        auto add_position = [&](const GridPosition &position)
        {
            if (std::find(estimate_positions.begin(), estimate_positions.end(), position) ==
                estimate_positions.end())
                estimate_positions.push_back(position);
        };
        for (const auto &robot : robots)
            add_position(robot.start);
        for (const auto &robot : robots)
        {
            if (robot.return_position)
                add_position(*robot.return_position);
        }
        for (const auto &task : tasks)
            add_position(task.location);
        std::vector<PathQueryProfile> estimate_profiles;
        estimate_profiles.reserve(robots.size());
        for (const auto &robot : robots)
        {
            const PathQueryProfile profile{
                std::max(0.0, robot.clearance_radius_m + robot.safety_margin_m),
                robot.nominal_speed_mps > 0.0 ? robot.nominal_speed_mps : _path_planner.options().nominal_speed_mps};
            const auto found = std::find_if(estimate_profiles.begin(), estimate_profiles.end(),
                                            [&](const auto &existing)
                                            {
                                                return existing.required_clearance_m == profile.required_clearance_m &&
                                                       existing.nominal_speed_mps == profile.nominal_speed_mps;
                                            });
            if (found == estimate_profiles.end())
                estimate_profiles.push_back(profile);
        }
        const auto precompute_started = std::chrono::steady_clock::now();
        _path_planner.precompute_estimates(estimate_positions, estimate_profiles);
        const auto precompute_finished = std::chrono::steady_clock::now();
        auto routes = build_routes(robots, tasks, service_ticks, _path_planner, _weights, 0U);
        const auto allocation_finished = std::chrono::steady_clock::now();

        OfflineMissionPlan result;
        result.allocation_seconds = std::chrono::duration<double>(
                                        allocation_finished - allocation_started)
                                        .count();
        result.estimate_precompute_seconds = std::chrono::duration<double>(
                                                 precompute_finished - precompute_started)
                                                 .count();
        result.allocation_path_stats = _path_planner.stats();
        result.routes = std::move(routes);
        const auto final_started = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < result.routes.size(); ++i)
            // Keep the configured planning resolution for reported loads and the
            // coordination timeline. plan_exact remains available for explicit
            // full-resolution refinement by callers.
            recompute_route(result.routes[i], robots[i], _path_planner, false);
        const auto final_finished = std::chrono::steady_clock::now();
        result.final_path_seconds = std::chrono::duration<double>(
                                        final_finished - final_started)
                                        .count();
        result.total_path_stats = _path_planner.stats();
        result.time_step_seconds = time_step;
        for (const auto &route : result.routes)
        {
            result.maximum_load_ticks = std::max(result.maximum_load_ticks, route.load_ticks());
            result.total_load_ticks += route.load_ticks();
        }
        if (coordinate_conflicts && !robots.empty())
        {
            const auto coordination_started = std::chrono::steady_clock::now();
            result.schedules = coordinate_multi_map_routes(
                _path_planner, robots, result.routes, &result.coordination_stats);
            const auto coordination_finished = std::chrono::steady_clock::now();
            result.coordination_seconds = std::chrono::duration<double>(
                                              coordination_finished - coordination_started)
                                              .count();
            result.shared_resources = _path_planner.options().shared_resources;
            extract_navigation_annotations(result, tasks);
        }
        return result;
    }
} // namespace capability_mission_planner::offline
