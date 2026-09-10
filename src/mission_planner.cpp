#include <capability_mission_planner/mission_planner.hpp>

#include <capability_mission_planner/cbs_coordinator.hpp>

#include <algorithm>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>

namespace capability_mission_planner
{
    namespace
    {
        void recompute_route(RobotRoute &route, const Robot &robot, const GridPathPlanner &planner)
        {
            route.travel_cost = 0;
            route.service_cost = 0;
            Location current = robot.start;
            for (const auto &stop : route.stops)
            {
                route.travel_cost += planner.distance(current, stop.location);
                route.service_cost += stop.service_duration;
                current = stop.location;
            }
            if (robot.return_home && !route.stops.empty())
                route.travel_cost += planner.distance(current, robot.start);
        }

        double objective(const std::vector<RobotRoute> &routes, const ObjectiveWeights &weights)
        {
            int maximum = 0;
            int total = 0;
            for (const auto &route : routes)
            {
                maximum = std::max(maximum, route.load());
                total += route.load();
            }
            return weights.maximum_load * maximum + weights.total_load * total;
        }

        std::vector<RobotRoute> insertion_options(const RobotRoute &source, std::size_t task_index, const AtomicTask &task)
        {
            std::vector<RobotRoute> options;
            for (std::size_t i = 0; i < source.stops.size(); ++i)
            {
                if (source.stops[i].location == task.location)
                {
                    auto merged = source;
                    merged.stops[i].task_indices.push_back(task_index);
                    merged.stops[i].service_duration += task.service_duration();
                    options.push_back(std::move(merged));
                    return options;
                }
            }

            for (std::size_t position = 0; position <= source.stops.size(); ++position)
            {
                auto inserted = source;
                inserted.stops.insert(
                    inserted.stops.begin() + static_cast<std::ptrdiff_t>(position),
                    RouteStop{task.location, {task_index}, task.service_duration()});
                options.push_back(std::move(inserted));
            }
            return options;
        }

        bool remove_task(RobotRoute &route, std::size_t task_index, const AtomicTask &task)
        {
            for (auto stop = route.stops.begin(); stop != route.stops.end(); ++stop)
            {
                const auto entry = std::find(stop->task_indices.begin(), stop->task_indices.end(), task_index);
                if (entry == stop->task_indices.end())
                    continue;
                stop->task_indices.erase(entry);
                stop->service_duration -= task.service_duration();
                if (stop->task_indices.empty())
                    route.stops.erase(stop);
                return true;
            }
            return false;
        }

        std::size_t compatible_robot_count(const std::vector<Robot> &robots, const AtomicTask &task)
        {
            return static_cast<std::size_t>(std::count_if(
                robots.begin(), robots.end(),
                [&task](const Robot &robot)
                { return is_compatible(robot, task); }));
        }

        void validate_inputs(const std::vector<Robot> &robots, const std::vector<AtomicTask> &tasks, const GridMap &map)
        {
            if (robots.empty() && !tasks.empty())
                throw std::invalid_argument("tasks cannot be assigned without robots");

            std::set<std::string> robot_ids;
            std::set<Location> robot_starts;
            for (const auto &robot : robots)
            {
                if (!robot_ids.insert(robot.id).second)
                    throw std::invalid_argument("duplicate robot id: " + robot.id);
                if (!map.traversable(robot.start))
                    throw std::invalid_argument("robot starts outside traversable grid: " + robot.id);
                if (!robot_starts.insert(robot.start).second)
                    throw std::invalid_argument("robots cannot share an initial location");
            }

            std::set<std::string> task_ids;
            for (const auto &task : tasks)
            {
                if (!task_ids.insert(task.id()).second)
                    throw std::invalid_argument("duplicate task id: " + task.id());
                if (!map.traversable(task.location))
                    throw std::invalid_argument("task is outside traversable grid: " + task.id());
                if (compatible_robot_count(robots, task) == 0U)
                {
                    throw std::invalid_argument("no capable robot for task " + task.id() + " (" + task.header.category() + ")");
                }
            }
        }
    } // namespace

    MissionPlanner::MissionPlanner(GridPathPlanner path_planner, ObjectiveWeights weights)
        : _path_planner(std::move(path_planner)), _weights(weights)
    {
        if (weights.maximum_load < 0.0 || weights.total_load < 0.0)
            throw std::invalid_argument("objective weights must be non-negative");
    }

    MissionPlan MissionPlanner::plan(const std::vector<Robot> &robots, const std::vector<AtomicTask> &tasks, bool coordinate_conflicts) const
    {
        validate_inputs(robots, tasks, _path_planner.map());

        std::vector<RobotRoute> routes(robots.size());
        for (std::size_t i = 0; i < routes.size(); ++i)
            routes[i].robot_index = i;

        std::vector<std::size_t> order(tasks.size());
        std::iota(order.begin(), order.end(), 0U);
        std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b)
        {
            const auto candidates_a = compatible_robot_count(robots, tasks[a]);
            const auto candidates_b = compatible_robot_count(robots, tasks[b]);
            if (candidates_a != candidates_b)
                return candidates_a < candidates_b;
            if (tasks[a].high_priority() != tasks[b].high_priority())
                return tasks[a].high_priority();
            return tasks[a].service_duration() > tasks[b].service_duration();
        });

        for (const auto task_index : order)
        {
            double best_score = std::numeric_limits<double>::infinity();
            std::size_t best_robot = robots.size();
            RobotRoute best_route;

            for (std::size_t r = 0; r < robots.size(); ++r)
            {
                if (!is_compatible(robots[r], tasks[task_index]))
                    continue;
                for (auto candidate : insertion_options(routes[r], task_index, tasks[task_index]))
                {
                    try
                    {
                        recompute_route(candidate, robots[r], _path_planner);
                    }
                    catch (const std::runtime_error &)
                    {
                        continue;
                    }
                    auto candidate_routes = routes;
                    candidate_routes[r] = candidate;
                    const double score = objective(candidate_routes, _weights);
                    if (score < best_score)
                    {
                        best_score = score;
                        best_robot = r;
                        best_route = std::move(candidate);
                    }
                }
            }

            if (best_robot == robots.size())
                throw std::runtime_error("no reachable capable robot for task " + tasks[task_index].id());
            routes[best_robot] = std::move(best_route);
        }

        // Improve the initial insertion solution by relocating one atomic task at a
        // time. This also permits route reordering when source and target are equal.
        for (std::size_t pass = 0; pass < tasks.size(); ++pass)
        {
            const double current_score = objective(routes, _weights);
            double best_score = current_score;
            std::vector<RobotRoute> best_routes;

            for (std::size_t source = 0; source < robots.size(); ++source)
            {
                std::vector<std::size_t> source_tasks;
                for (const auto &stop : routes[source].stops)
                {
                    source_tasks.insert(source_tasks.end(), stop.task_indices.begin(), stop.task_indices.end());
                }
                for (const auto task_index : source_tasks)
                {
                    auto without = routes;
                    if (!remove_task(without[source], task_index, tasks[task_index]))
                        continue;
                    recompute_route(without[source], robots[source], _path_planner);

                    for (std::size_t target = 0; target < robots.size(); ++target)
                    {
                        if (!is_compatible(robots[target], tasks[task_index]))
                            continue;
                        for (auto candidate : insertion_options(without[target], task_index, tasks[task_index]))
                        {
                            recompute_route(candidate, robots[target], _path_planner);
                            auto trial = without;
                            trial[target] = std::move(candidate);
                            const auto score = objective(trial, _weights);
                            if (score + 1e-9 < best_score)
                            {
                                best_score = score;
                                best_routes = std::move(trial);
                            }
                        }
                    }
                }
            }

            if (best_routes.empty())
                break;
            routes = std::move(best_routes);
        }

        MissionPlan result;
        result.routes = std::move(routes);
        for (const auto &route : result.routes)
        {
            result.maximum_load = std::max(result.maximum_load, route.load());
            result.total_load += route.load();
        }

        if (coordinate_conflicts && !robots.empty())
        {
            CbsCoordinator coordinator(_path_planner.map());
            result.schedules = coordinator.coordinate(robots, result.routes);
        }
        return result;
    }
} // namespace capability_mission_planner
