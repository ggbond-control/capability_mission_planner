#include <capability_mission_planner/offline/offline_map_planner.hpp>

#include "coordination/multi_map_coordinator.hpp"

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
            double reference_speed_mps = MultiMapPathPlanner::allocation_reference_speed_mps;

            int edge(const GridPosition &from, const GridPosition &to) const
            {
                const double clearance = std::max(0.0, robot->clearance_radius_m + robot->safety_margin_m);
                const double speed = robot->nominal_speed_mps > 0.0 ? robot->nominal_speed_mps : planner->options().nominal_speed_mps;
                return planner->estimate_allocation_ticks(from, to, robot->capabilities, clearance, speed,
                                                         reference_speed_mps);
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

            RouteState removed(std::size_t task_index, int service_ticks,
                               const std::vector<MappedTask> &tasks) const
            {
                RouteState result = *this;
                for (std::size_t position = 0; position < route.stops.size(); ++position)
                {
                    auto &stop = result.route.stops[position];
                    const auto found = std::find(stop.task_indices.begin(), stop.task_indices.end(), task_index);
                    if (found == stop.task_indices.end())
                        continue;
                    stop.task_indices.erase(found);
                    stop.service_ticks -= service_ticks;
                    result.route.service_ticks -= service_ticks;
                    if (!stop.task_indices.empty())
                    {
                        stop.position_tolerance_m = 0.0;
                        for (const auto index : stop.task_indices)
                            stop.position_tolerance_m = std::max(stop.position_tolerance_m, tasks[index].position_tolerance_m);
                        return result;
                    }
                    result.route.stops.erase(result.route.stops.begin() + static_cast<std::ptrdiff_t>(position));
                    if (result.route.stops.empty())
                    {
                        result.edges.clear();
                        result.route.travel_ticks = 0;
                        return result;
                    }
                    const auto previous = position == 0U ? robot->start : route.stops[position - 1U].location;
                    const bool has_next = position + 1U < route.stops.size() || robot->return_position.has_value();
                    const int joined = has_next ? edge(previous, position + 1U < route.stops.size()
                        ? route.stops[position + 1U].location : *robot->return_position) : 0;
                    result.route.travel_ticks += joined - edges[position] -
                        (position + 1U < edges.size() ? edges[position + 1U] : 0);
                    result.edges.erase(result.edges.begin() + static_cast<std::ptrdiff_t>(position));
                    if (position < result.edges.size())
                        result.edges[position] = joined;
                    return result;
                }
                throw std::logic_error("cannot remove an unassigned task");
            }
        };

        std::vector<std::size_t> task_counts(const std::vector<MappedRobotRoute> &routes)
        {
            std::vector<std::size_t> counts;
            for (const auto &route : routes)
            {
                std::size_t count = 0;
                for (const auto &stop : route.stops)
                    count += stop.task_indices.size();
                counts.push_back(count);
            }
            return counts;
        }

        std::pair<int, long long> loads(const std::vector<MappedRobotRoute> &routes)
        {
            int maximum = 0;
            long long total = 0;
            for (const auto &route : routes)
            {
                maximum = std::max(maximum, route.load_ticks());
                total += route.load_ticks();
            }
            return {maximum, total};
        }

        bool within_slack(const std::pair<int, long long> &candidate,
                          const std::pair<int, long long> &reference, double slack)
        {
            return candidate.first <= reference.first * (1.0 + slack) + 1e-9 &&
                   candidate.second <= reference.second * (1.0 + slack) + 1e-9;
        }

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
            const ObjectiveWeights &weights, const AllocationOptions &options,
            AllocationStats &stats, std::vector<MappedRobotRoute> &before_balancing,
            double reference_speed_mps)
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
                states[i].reference_speed_mps = reference_speed_mps;
                states[i].rebuild();
            }
            std::set<std::size_t> remaining;
            for (std::size_t i = 0; i < tasks.size(); ++i)
                remaining.insert(i);
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

            stats.initial_task_counts = task_counts(routes);
            const auto started = std::chrono::steady_clock::now();
            const auto exhausted = [&]()
            {
                if (stats.candidate_evaluations >= options.max_candidate_evaluations ||
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >= options.budget_seconds)
                    stats.budget_exhausted = true;
                return stats.budget_exhausted;
            };
            // Use edge deltas and retain only the winning pair of routes, rather
            // than rebuilding/copying every robot's route for every trial.
            const auto improve = [&](bool balancing, const std::pair<int, long long> &reference)
            {
                for (std::size_t pass = 0; pass < options.max_passes && !exhausted(); ++pass)
                {
                    const auto counts = task_counts(routes);
                    double best = objective(routes, weights);
                    long long best_count_delta = 0;
                    std::optional<std::pair<RouteState, RouteState>> winner;
                    std::size_t winner_source = 0, winner_target = 0;
                    for (std::size_t source = 0; source < robots.size() && !exhausted(); ++source)
                    {
                        for (const auto &stop : routes[source].stops)
                        {
                            if (exhausted()) break;
                            for (const auto ti : stop.task_indices)
                            {
                                if (exhausted()) break;
                                const auto without = states[source].removed(ti, service_ticks[ti], tasks);
                                for (std::size_t target = 0; target < robots.size() && !exhausted(); ++target)
                                {
                                    if (!compatible(robots[target], tasks[ti]) ||
                                        (balancing && (target == source || counts[source] <= counts[target] + 1U)))
                                        continue;
                                    const auto &base = target == source ? without : states[target];
                                    const long long count_delta = target == source ? 0 :
                                        2LL * (static_cast<long long>(counts[target]) - static_cast<long long>(counts[source]) + 1LL);
                                    // Iterate incrementally, so a large neighborhood
                                    // cannot bypass either compute budget.
                                    for (std::size_t position = 0; position <= base.route.stops.size(); ++position)
                                    {
                                        if (exhausted()) break;
                                        const auto existing = std::find_if(base.route.stops.begin(), base.route.stops.end(),
                                            [&](const auto &s) { return s.location == tasks[ti].location; });
                                        if (existing != base.route.stops.end() &&
                                            position != static_cast<std::size_t>(existing - base.route.stops.begin()))
                                            continue;
                                        ++stats.candidate_evaluations;
                                        auto candidate = existing == base.route.stops.end()
                                            ? base.inserted(position, {tasks[ti].location, {ti}, service_ticks[ti], tasks[ti].position_tolerance_m})
                                            : base.merged(position, ti, service_ticks[ti], tasks[ti].position_tolerance_m);
                                        int maximum = 0;
                                        long long total = 0;
                                        for (std::size_t r = 0; r < routes.size(); ++r)
                                        {
                                            const int load = r == target ? candidate.route.load_ticks() :
                                                r == source ? without.route.load_ticks() : routes[r].load_ticks();
                                            maximum = std::max(maximum, load);
                                            total += load;
                                        }
                                        const double score = weights.maximum_load * maximum + weights.total_load * total;
                                        const bool better = balancing
                                            ? within_slack({maximum, total}, reference, options.balance_load_slack) &&
                                                (count_delta < best_count_delta ||
                                                 (count_delta == best_count_delta && score + 1e-9 < best))
                                            : score + 1e-9 < best;
                                        if (better)
                                        {
                                            best = score;
                                            best_count_delta = count_delta;
                                            winner = {without, std::move(candidate)};
                                            winner_source = source;
                                            winner_target = target;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if (!winner) break;
                    states[winner_source] = std::move(winner->first);
                    states[winner_target] = std::move(winner->second);
                    routes[winner_source] = states[winner_source].route;
                    routes[winner_target] = states[winner_target].route;
                    if (balancing) ++stats.balancing_moves;
                    else ++stats.relocation_moves;
                }
            };
            improve(false, {});
            before_balancing = routes;
            if (options.balance_task_counts)
                improve(true, loads(routes));
            stats.local_search_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
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
        ObjectiveWeights weights, AllocationOptions allocation)
        : _path_planner(std::move(path_planner)), _weights(weights), _allocation(allocation)
    {
        if (weights.maximum_load < 0.0 || weights.total_load < 0.0)
            throw std::invalid_argument("objective weights must be non-negative");
        if (!std::isfinite(allocation.budget_seconds) || allocation.budget_seconds <= 0.0 ||
            allocation.max_candidate_evaluations == 0U ||
            !std::isfinite(allocation.balance_load_slack) || allocation.balance_load_slack < 0.0)
            throw std::invalid_argument("allocation requires a positive finite budget and non-negative finite load slack");
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
        // Same-speed fleets retain their existing estimates. A mixed fleet
        // shares the slowest member's profile, then scales once per leg.
        double reference_speed = std::numeric_limits<double>::infinity();
        for (const auto &robot : robots)
            reference_speed = std::min(reference_speed, robot.nominal_speed_mps > 0.0
                ? robot.nominal_speed_mps : _path_planner.options().nominal_speed_mps);
        if (robots.empty())
            reference_speed = MultiMapPathPlanner::allocation_reference_speed_mps;
        for (const auto &robot : robots)
        {
            const PathQueryProfile profile{
                std::max(0.0, robot.clearance_radius_m + robot.safety_margin_m),
                reference_speed};
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
        AllocationStats allocation_stats;
        std::vector<MappedRobotRoute> before_balancing;
        auto routes = build_routes(robots, tasks, service_ticks, _path_planner, _weights,
                                   _allocation, allocation_stats, before_balancing, reference_speed);
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
        // The final allocation guard must use the same speed-adjusted scores as
        // insertion and relocation, before replacing them with timeline ticks.
        if (allocation_stats.balancing_moves > 0U)
        {
            if (!within_slack(loads(result.routes), loads(before_balancing), _allocation.balance_load_slack))
            {
                result.routes = std::move(before_balancing);
                allocation_stats.balance_reverted = true;
            }
        }
        for (std::size_t i = 0; i < result.routes.size(); ++i)
            // Exported travel/load ticks and coordination remain on the existing
            // coarse timeline; allocation scores are deliberately not exported
            // in their place. Navigation arrival / holding semantics stay intact.
            recompute_route(result.routes[i], robots[i], _path_planner, false);
        allocation_stats.final_task_counts = task_counts(result.routes);
        result.allocation_stats = std::move(allocation_stats);
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
