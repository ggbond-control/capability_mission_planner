#include <capability_mission_planner/cbs_coordinator.hpp>
#include <capability_mission_planner/mission_planner.hpp>

#include <algorithm>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>

using namespace capability_mission_planner;

namespace
{

    void require(bool condition, const std::string &message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    std::vector<Robot> four_robots()
    {
        return {
            {"a", {0, 0}, {"fire", "camera"}, true},
            {"b", {11, 0}, {"camera", "thermal"}, true},
            {"c", {0, 11}, {"fire", "thermal"}, true},
            {"d", {11, 11}, {"fire", "camera", "thermal"}, true}};
    }

    std::vector<AtomicTask> forty_tasks_at_twenty_points()
    {
        std::vector<AtomicTask> tasks;
        for (int i = 0; i < 20; ++i)
        {
            const Location point{1 + (i % 10), 1 + 3 * (i / 10)};
            tasks.push_back(make_task(
                "P" + std::to_string(i) + "-camera", point, {"camera"},
                "gimbal_photo", 1, i % 7 == 0));
            const auto capability = i % 2 == 0 ? "fire" : "thermal";
            tasks.push_back(make_task(
                "P" + std::to_string(i) + "-" + capability, point, {capability},
                capability, 2));
        }
        return tasks;
    }

    void test_task_metadata()
    {
        const auto task = make_task(
            "fire-01", {2, 3}, {"fire"}, "fire_suppression", 7, true, 5);
        require(task.id() == "fire-01", "task booking id was not retained");
        require(task.header.category() == "fire_suppression", "task category missing");
        require(task.service_duration() == 7, "task duration was not retained");
        require(task.high_priority(), "task priority was not retained");
        require(task.booking->labels() == std::vector<std::string>{"fire_suppression"},
                "task booking label missing");
    }

    void test_astar_uses_obstacles()
    {
        GridPathPlanner planner(GridMap(5, 3, {{{2, 0}, {2, 1}}}));
        const auto path = planner.plan({0, 0}, {4, 0});
        require(path.cost == 8, "A* did not take the expected obstacle detour");
        require(std::none_of(path.states.begin(), path.states.end(), [](const Location &p)
                             { return p == Location{2, 0} || p == Location{2, 1}; }),
                "A* crossed an obstacle");
    }

    void test_variable_multi_robot_multi_task_assignment()
    {
        const auto robots = four_robots();
        const auto tasks = forty_tasks_at_twenty_points();
        MissionPlanner planner(GridPathPlanner(GridMap(12, 12)));
        const auto result = planner.plan(robots, tasks, true);

        require(result.routes.size() == 4, "route count does not match robot count");
        std::set<std::size_t> assigned;
        bool merged_same_point_tasks = false;
        std::size_t robots_with_work = 0;
        for (const auto &route : result.routes)
        {
            if (!route.stops.empty())
                ++robots_with_work;
            const auto &robot = robots[route.robot_index];
            for (const auto &stop : route.stops)
            {
                merged_same_point_tasks = merged_same_point_tasks || stop.task_indices.size() > 1;
                for (const auto task_index : stop.task_indices)
                {
                    require(task_index < tasks.size(), "invalid task index in route");
                    require(assigned.insert(task_index).second, "task was assigned more than once");
                    require(is_compatible(robot, tasks[task_index]), "task assigned to incapable robot");
                    require(tasks[task_index].location == stop.location, "task assigned at wrong stop");
                }
            }
        }
        require(assigned.size() == tasks.size(), "not every atomic task was assigned");
        require(merged_same_point_tasks, "same-point tasks were never merged");
        require(robots_with_work == robots.size(), "load objective left a robot unused");
        require(result.maximum_load > 0 && result.total_load >= result.maximum_load,
                "invalid mission load summary");
        require(result.schedules.size() == robots.size(), "CBS schedules were not generated");

        const auto horizon = std::max_element(
                                 result.schedules.begin(), result.schedules.end(),
                                 [](const auto &a, const auto &b)
                                 { return a.size() < b.size(); })
                                 ->size();
        auto at = [&result](std::size_t agent, std::size_t time)
        {
            const auto &schedule = result.schedules[agent];
            return schedule[std::min(time, schedule.size() - 1)].location;
        };
        for (std::size_t t = 0; t < horizon; ++t)
        {
            for (std::size_t a = 0; a < robots.size(); ++a)
            {
                for (std::size_t b = a + 1; b < robots.size(); ++b)
                {
                    require(at(a, t) != at(b, t), "end-to-end plan has a vertex conflict");
                    if (t + 1 < horizon)
                    {
                        require(!(at(a, t) == at(b, t + 1) && at(b, t) == at(a, t + 1)),
                                "end-to-end plan has an opposing edge conflict");
                    }
                }
            }
        }

        for (const auto &route : result.routes)
        {
            std::size_t cursor = 0;
            for (const auto &stop : route.stops)
            {
                while (cursor < result.schedules[route.robot_index].size() &&
                       result.schedules[route.robot_index][cursor].location != stop.location)
                {
                    ++cursor;
                }
                require(cursor < result.schedules[route.robot_index].size(),
                        "CBS schedule did not preserve route stop order");
            }
        }
    }

    void test_unserviceable_task_is_rejected()
    {
        const std::vector<Robot> robots{{"camera-only", {0, 0}, {"camera"}, true}};
        const std::vector<AtomicTask> tasks{
            make_task("fire", {1, 0}, {"fire"}, "fire", 1)};
        MissionPlanner planner(GridPathPlanner(GridMap(3, 3)));
        bool threw = false;
        try
        {
            (void)planner.plan(robots, tasks, false);
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        require(threw, "task without a capable robot was accepted");
    }

    void test_cbs_removes_vertex_and_edge_conflicts()
    {
        const GridMap map(3, 2);
        const std::vector<Robot> robots{
            {"a", {0, 0}, {"move"}, false},
            {"b", {2, 0}, {"move"}, false}};
        std::vector<RobotRoute> routes(2);
        routes[0] = RobotRoute{0, {{{2, 0}, {0}, 0}}, 2, 0};
        routes[1] = RobotRoute{1, {{{0, 0}, {1}, 0}}, 2, 0};

        const auto schedules = CbsCoordinator(map).coordinate(robots, routes);
        require(schedules.size() == 2, "CBS schedule count mismatch");
        const auto horizon = std::max(schedules[0].size(), schedules[1].size());
        auto at = [&schedules](std::size_t agent, std::size_t time)
        {
            const auto &schedule = schedules[agent];
            return schedule[std::min(time, schedule.size() - 1)].location;
        };
        for (std::size_t t = 0; t < horizon; ++t)
        {
            require(at(0, t) != at(1, t), "CBS left a vertex conflict");
            if (t + 1 < horizon)
            {
                require(!(at(0, t) == at(1, t + 1) && at(1, t) == at(0, t + 1)),
                        "CBS left an opposing edge conflict");
            }
        }
        require(at(0, horizon - 1) == Location{2, 0}, "robot a missed its task point");
        require(at(1, horizon - 1) == Location{0, 0}, "robot b missed its task point");
    }

    void test_idle_robot_still_participates_in_cbs()
    {
        const GridMap map(3, 2);
        const std::vector<Robot> robots{
            {"idle", {1, 0}, {}, false},
            {"worker", {0, 0}, {"camera"}, false}};
        std::vector<RobotRoute> routes(2);
        routes[0] = RobotRoute{0, {}, 0, 0};
        routes[1] = RobotRoute{1, {{{2, 0}, {0}, 0}}, 2, 0};

        const auto schedules = CbsCoordinator(map).coordinate(robots, routes);
        require(schedules.size() == 2, "idle robot CBS schedule is missing");
        const auto horizon = std::max(schedules[0].size(), schedules[1].size());
        auto at = [&schedules](std::size_t agent, std::size_t time)
        {
            const auto &schedule = schedules[agent];
            return schedule[std::min(time, schedule.size() - 1)].location;
        };
        for (std::size_t t = 0; t < horizon; ++t)
            require(at(0, t) != at(1, t), "idle robot was ignored by CBS");
        require(at(0, horizon - 1) == robots[0].start,
                "idle robot did not return to its original location");
        require(at(1, horizon - 1) == Location{2, 0},
                "worker could not pass the idle robot");
    }

} // namespace

int main()
{
    try
    {
        test_task_metadata();
        test_astar_uses_obstacles();
        test_variable_multi_robot_multi_task_assignment();
        test_unserviceable_task_is_rejected();
        test_cbs_removes_vertex_and_edge_conflicts();
        test_idle_robot_still_participates_in_cbs();
        std::cout << "All capability mission planner tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Test failure: " << error.what() << '\n';
        return 1;
    }
}
