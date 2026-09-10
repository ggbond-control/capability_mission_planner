#include <capability_mission_planner/offline_planner_config.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace capability_mission_planner::offline;

namespace
{

    void require(bool condition, const std::string &message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    TimedMapState state_at(const std::vector<TimedMapState> &schedule, std::size_t tick)
    {
        return tick < schedule.size() ? schedule[tick] : schedule.back();
    }

    void require_conflict_free(const OfflineMissionPlan &plan)
    {
        std::size_t horizon = 0;
        for (const auto &schedule : plan.schedules)
        {
            require(!schedule.empty(), "robot schedule is empty");
            horizon = std::max(horizon, schedule.size());
        }
        for (std::size_t tick = 0; tick < horizon; ++tick)
        {
            for (std::size_t a = 0; a < plan.schedules.size(); ++a)
            {
                for (std::size_t b = a + 1U; b < plan.schedules.size(); ++b)
                {
                    const auto first = state_at(plan.schedules[a], tick);
                    const auto second = state_at(plan.schedules[b], tick);
                    require(first.position != second.position, "vertex conflict at tick " +
                                                                   std::to_string(tick));
                    require(first.transition_id.empty() || first.transition_id != second.transition_id,
                            "transition conflict at tick " + std::to_string(tick));
                    if (tick + 1U >= horizon)
                        continue;
                    const auto first_next = state_at(plan.schedules[a], tick + 1U);
                    const auto second_next = state_at(plan.schedules[b], tick + 1U);
                    require(!(first.position == second_next.position &&
                              second.position == first_next.position),
                            "opposing edge conflict at tick " + std::to_string(tick));
                }
            }
        }
    }

    void require_navigation_checkpoints(const OfflineMissionPlan &plan)
    {
        for (const auto &checkpoints : plan.navigation_checkpoints)
        {
            require(!checkpoints.empty(), "navigation checkpoints are empty");
            require(checkpoints.front().type == NavigationCheckpointType::Start,
                    "navigation checkpoints do not start with start");
            require(checkpoints.back().type == NavigationCheckpointType::Finish,
                    "navigation checkpoints do not end with finish");
            for (std::size_t i = 1; i < checkpoints.size(); ++i)
                require(!(checkpoints[i - 1U].type == NavigationCheckpointType::Turn &&
                          checkpoints[i].type == NavigationCheckpointType::Turn),
                        "adjacent turn checkpoints were not merged");
        }
    }

} // namespace

int main(int argc, char *argv[])
{
    try
    {
        if (argc != 2)
            throw std::invalid_argument("expected one scenario config path");
        const auto root = YAML::LoadFile(argv[1]);
        const auto expected = root["test_expectations"];
        require(expected && expected.IsMap(), "test_expectations is required");

        auto request = OfflinePlannerConfigLoader::load(argv[1]);
        require(request.robots.size() == expected["robots"].as<std::size_t>(),
                "unexpected robot count");
        require(request.tasks.size() == expected["tasks"].as<std::size_t>(),
                "unexpected task count");

        OfflineMissionPlanner planner{
            MultiMapPathPlanner(request.bundle, request.traversal), request.objective};
        const auto plan = planner.plan(
            request.robots, request.tasks, request.coordinate_conflicts);

        std::vector<int> assignments(request.tasks.size(), 0);
        std::size_t idle = 0;
        std::size_t merged_stops = 0;
        for (const auto &route : plan.routes)
        {
            if (route.stops.empty())
                ++idle;
            for (const auto &stop : route.stops)
            {
                if (stop.task_indices.size() > 1U)
                    ++merged_stops;
                for (const auto task_index : stop.task_indices)
                {
                    require(task_index < request.tasks.size(), "invalid task index");
                    ++assignments[task_index];
                    for (const auto &capability : request.tasks[task_index].requirements)
                    {
                        require(request.robots[route.robot_index].capabilities.count(capability) != 0U,
                                request.tasks[task_index].id() + " assigned to incapable robot " +
                                    request.robots[route.robot_index].id);
                    }
                }
            }
        }
        require(std::all_of(assignments.begin(), assignments.end(),
                            [](int count)
                            { return count == 1; }),
                "tasks are not assigned exactly once");
        require(idle == expected["idle_robots"].as<std::size_t>(),
                "unexpected idle robot count");
        require(merged_stops >= expected["minimum_merged_stops"].as<std::size_t>(),
                "too few merged same-location stops");
        require(plan.schedules.size() == request.robots.size(), "schedules are missing");
        require_conflict_free(plan);
        require_navigation_checkpoints(plan);

        std::cout << "scenario passed: robots=" << request.robots.size()
                  << " tasks=" << request.tasks.size() << " idle=" << idle
                  << " merged_stops=" << merged_stops << '\n';
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "scenario failed: " << error.what() << '\n';
        return 1;
    }
}
