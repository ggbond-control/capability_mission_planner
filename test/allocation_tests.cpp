#include <capability_mission_planner/offline/offline_map_planner.hpp>
#include <capability_mission_planner/offline/offline_planner_config.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace capability_mission_planner::offline;

namespace
{
    void require(bool condition, const char *message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    std::shared_ptr<MultiMapBundle> bundle()
    {
        auto result = std::make_shared<MultiMapBundle>();
        MapLayer map;
        map.id = "test";
        map.width = map.height = 40;
        map.resolution = 0.1;
        map.traversable.assign(1600, 1);
        map.clearance_m.assign(1600, 10.0F);
        map.inflated_cost.assign(1600, 0);
        result->maps.emplace(map.id, map);
        return result;
    }

    std::vector<MappedRobot> robots()
    {
        return {{"a", {"test", 5, 10}, {"device"}, std::nullopt},
                {"b", {"test", 25, 10}, {"device"}, std::nullopt}};
    }

    std::vector<MappedTask> tasks()
    {
        std::vector<MappedTask> result;
        for (int i = 0; i < 28; ++i)
            result.push_back(make_mapped_task(std::to_string(i), {"test", 15, 10},
                {"device"}, "inspection", i == 0 ? 100 : 1));
        return result;
    }

    void verify(const OfflineMissionPlan &plan, const std::vector<MappedRobot> &rs,
                const std::vector<MappedTask> &ts, const MultiMapPathPlanner &planner)
    {
        std::vector<int> seen(ts.size(), 0);
        for (std::size_t r = 0; r < rs.size(); ++r)
        {
            int service = 0, travel = 0;
            auto previous = rs[r].start;
            for (const auto &stop : plan.routes[r].stops)
            {
                double tolerance = 0.0;
                travel += planner.plan(previous, stop.location, rs[r].capabilities,
                    std::max(0.0, rs[r].clearance_radius_m + rs[r].safety_margin_m), rs[r].nominal_speed_mps).travel_ticks;
                previous = stop.location;
                for (auto ti : stop.task_indices)
                {
                    require(ti < ts.size(), "invalid task index");
                    ++seen[ti];
                    require(stop.location == ts[ti].location, "task location changed");
                    tolerance = std::max(tolerance, ts[ti].position_tolerance_m);
                    service += static_cast<int>(std::ceil(ts[ti].service_duration_seconds() / plan.time_step_seconds - 1e-9));
                    for (const auto &capability : ts[ti].requirements)
                        require(rs[r].capabilities.count(capability) != 0U, "incapable assignment");
                }
                require(tolerance == stop.position_tolerance_m, "removal retained another task's tolerance");
            }
            if (rs[r].return_position && !plan.routes[r].stops.empty())
                travel += planner.plan(previous, *rs[r].return_position, rs[r].capabilities,
                    std::max(0.0, rs[r].clearance_radius_m + rs[r].safety_margin_m), rs[r].nominal_speed_mps).travel_ticks;
            require(service == plan.routes[r].service_ticks, "removal/merge lost service duration");
            require(travel == plan.routes[r].travel_ticks, "edge delta changed final travel duration");
        }
        require(std::all_of(seen.begin(), seen.end(), [](int n) { return n == 1; }), "tasks lost or duplicated");
    }

    void large_allocation_and_load_bound()
    {
        auto maps = bundle();
        auto rs = robots();
        const auto ts = tasks();
        AllocationOptions options;
        options.balance_task_counts = false;
        auto timed = OfflineMissionPlanner(MultiMapPathPlanner(maps), {}, options).plan(rs, ts, false);
        require(timed.allocation_stats.candidate_evaluations > 0, "large mission skipped local search");
        options.balance_task_counts = true;
        const auto balanced = OfflineMissionPlanner(MultiMapPathPlanner(maps), {}, options).plan(rs, ts, false);
        verify(balanced, rs, ts, MultiMapPathPlanner(maps));
        const auto spread = [](const auto &counts) { return *std::max_element(counts.begin(), counts.end()) - *std::min_element(counts.begin(), counts.end()); };
        require(balanced.allocation_stats.balancing_moves > 0, "fairness search did not run");
        require(spread(balanced.allocation_stats.final_task_counts) < spread(timed.allocation_stats.final_task_counts), "task counts did not improve");
        require(balanced.maximum_load_ticks <= timed.maximum_load_ticks * 1.1 + 1e-9 &&
                balanced.total_load_ticks <= timed.total_load_ticks * 1.1 + 1e-9, "fairness exceeded load bound");
        require(spread(balanced.allocation_stats.final_task_counts) > 0, "count equality overrode heavy service constraint");
        // A merged waypoint still contains multiple mission tasks. Balance the
        // task IDs, not the number of navigation stops.
        require(balanced.routes[0].stops.size() == 1 && balanced.routes[1].stops.size() == 1, "same-location merge lost");

        rs[1].capabilities.clear();
        const auto restricted = OfflineMissionPlanner(MultiMapPathPlanner(maps)).plan(rs, ts, false);
        verify(restricted, rs, ts, MultiMapPathPlanner(maps));
        require(restricted.routes[1].stops.empty(), "fairness bypassed capability restriction");

        options.balance_load_slack = 0.0;
        const auto strict = OfflineMissionPlanner(MultiMapPathPlanner(maps), {}, options).plan(robots(), ts, false);
        require(strict.maximum_load_ticks <= timed.maximum_load_ticks && strict.total_load_ticks <= timed.total_load_ticks,
                "zero slack increased route load");
    }

    void removal_return_and_budgets()
    {
        auto maps = bundle();
        auto rs = robots();
        rs[0].return_position = rs[0].start;
        rs[1].return_position = rs[1].start;
        auto ts = tasks();
        for (std::size_t i = 0; i < ts.size(); ++i)
        {
            ts[i].location = {"test", 2 + static_cast<int>(i % 30), 5 + static_cast<int>(i / 4)};
            ts[i].position_tolerance_m = i * .01;
        }
        AllocationOptions options;
        const auto full = OfflineMissionPlanner(MultiMapPathPlanner(maps), {}, options).plan(rs, ts, false);
        verify(full, rs, ts, MultiMapPathPlanner(maps));
        options.max_candidate_evaluations = 1;
        const auto limited = OfflineMissionPlanner(MultiMapPathPlanner(maps), {}, options).plan(rs, ts, false);
        require(limited.allocation_stats.candidate_evaluations == 1 && limited.allocation_stats.budget_exhausted,
                "candidate budget did not bound search");
        verify(limited, rs, ts, MultiMapPathPlanner(maps));
        options.budget_seconds = 1e-12;
        const auto expired = OfflineMissionPlanner(MultiMapPathPlanner(maps), {}, options).plan(rs, ts, false);
        require(expired.allocation_stats.budget_exhausted && expired.allocation_stats.candidate_evaluations == 0,
                "time budget did not stop search");
        verify(expired, rs, ts, MultiMapPathPlanner(maps));
        options.budget_seconds = 1;
        options.max_passes = 0;
        const auto disabled = OfflineMissionPlanner(MultiMapPathPlanner(maps), {}, options).plan(rs, ts, false);
        require(disabled.allocation_stats.candidate_evaluations == 0, "max_passes zero did not disable search");
        require(disabled.allocation_stats.initial_task_counts == disabled.allocation_stats.final_task_counts,
                "disabled search changed allocation");
        verify(disabled, rs, ts, MultiMapPathPlanner(maps));
        const auto empty = OfflineMissionPlanner(MultiMapPathPlanner(maps)).plan({}, {}, false);
        require(empty.routes.empty(), "empty mission changed");
        rs.resize(1);
        const auto single = OfflineMissionPlanner(MultiMapPathPlanner(maps)).plan(rs, ts, false);
        verify(single, rs, ts, MultiMapPathPlanner(maps));
    }

    void allocation_speed_estimates()
    {
        auto maps = bundle();
        TraversalOptions traversal;
        traversal.obstacle_cost_weight = 0.0;
        traversal.nominal_speed_mps = 1.5;
        MultiMapPathPlanner planner(maps, traversal);
        const GridPosition start{"test", 5, 5}, straight{"test", 35, 5}, diagonal{"test", 35, 35};
        planner.precompute_estimates({start, straight, diagonal}, {{0.0, 1.0}});
        for (const auto &entry : std::vector<std::pair<double, int>>{{1.0, 30}, {1.5, 20}, {2.0, 15}})
        {
            require(planner.estimate_allocation_ticks(start, straight, {}, 0.0, entry.first) == entry.second,
                    "straight allocation lost speed ratio");
            require(planner.estimate_allocation_ticks(start, diagonal, {}, 0.0, entry.first) == entry.second * 2,
                    "diagonal allocation applied speed correction twice");
            require(planner.estimate_allocation_ticks(start, start, {}, 0.0, entry.first) == 0,
                    "zero-distance allocation gained travel time");
        }
        require(planner.estimate_allocation_ticks(start, straight, {}, 0.0) == 20,
                "allocation ignored configured fallback speed");
        require(planner.estimate_allocation_ticks(start, {"test", 6, 5}, {}, 0.0, 1.5) == 1,
                "short leg rounded to zero");
        // The allocation estimate is not the route timeline. Preserve the old
        // cardinal/diagonal timing and every step's timestamp for coordination.
        require(planner.plan(start, straight, {}, 0.0, 1.5).travel_ticks == 30 &&
                planner.plan(start, diagonal, {}, 0.0, 1.5).travel_ticks == 30,
                "allocation correction changed coordination path timing");
        require(planner.stats().distance_field_searches == 3,
                "speed scaling added per-speed distance fields");
        for (double speed : {0.5, 1.0, 1.5, 2.0})
            require(planner.estimate_allocation_ticks(start, diagonal, {}, 0.0, speed, speed) ==
                    planner.estimate_distance(start, diagonal, {}, 0.0, speed),
                    "same-speed reference changed existing estimate");
        for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::quiet_NaN()})
        {
            bool rejected = false;
            try { (void)planner.estimate_allocation_ticks(start, straight, {}, 0.0, 1.5, invalid); }
            catch (const std::invalid_argument &) { rejected = true; }
            require(rejected, "invalid allocation reference speed accepted");
        }
        bool overflow = false;
        try { (void)planner.estimate_allocation_ticks(start, straight, {}, 0.0, 1e-100); }
        catch (const std::overflow_error &) { overflow = true; }
        require(overflow, "allocation tick overflow accepted");

        auto other = maps->maps.at("test");
        other.id = "other";
        maps->maps.emplace(other.id, other);
        MapTransition portal;
        portal.id = "stairs";
        portal.type = "stairs";
        portal.from_map = "test";
        portal.to_map = "other";
        portal.from_cell = {"test", 25, 5};
        portal.to_cell = {"other", 5, 5};
        maps->transitions.push_back(portal);
        MultiMapPathPlanner multi(maps, traversal);
        // 3 m walking plus 8 s stairs and 2 s map switching. Only walking scales.
        const GridPosition goal{"other", 15, 5};
        require(multi.estimate_allocation_ticks(start, goal, {"stairs"}, 0.0, 1.0) == 130 &&
                multi.estimate_allocation_ticks(start, goal, {"stairs"}, 0.0, 1.5) == 120 &&
                multi.estimate_allocation_ticks(start, goal, {"stairs"}, 0.0, 2.0) == 115,
                "walking speed scaled fixed transition duration");
        bool rejected = false;
        try { (void)multi.estimate_allocation_ticks(start, goal, {}, 0.0, 2.0); }
        catch (const std::runtime_error &) { rejected = true; }
        require(rejected, "reference cache bypassed transition capability");
    }

    void speed_aware_assignment()
    {
        const auto maps = bundle();
        for (bool diagonal : {false, true})
            for (double speed : {1.5, 2.0})
                for (bool return_home : {false, true})
                {
                    auto rs = robots();
                    rs[0].start = {"test", 5, 5};
                    rs[1].start = {"test", 25, diagonal ? 25 : 5};
                    rs[0].nominal_speed_mps = 1.0;
                    rs[1].nominal_speed_mps = speed;
                    if (return_home)
                        for (auto &robot : rs) robot.return_position = robot.start;
                    const std::vector<MappedTask> ts{make_mapped_task("equal-distance",
                        {"test", 15, diagonal ? 15 : 5}, {"device"}, "inspection", 7)};
                    const auto plan = OfflineMissionPlanner(MultiMapPathPlanner(maps)).plan(rs, ts, true);
                    require(plan.routes[0].stops.empty() && plan.routes[1].stops.size() == 1,
                            "equal-distance assignment did not favor faster robot");
                    require(plan.routes[1].service_ticks == 70, "speed scaled task service time");
                    verify(plan, rs, ts, MultiMapPathPlanner(maps));
                    require(plan.schedules[1].back().tick == plan.routes[1].load_ticks(),
                            "allocation score leaked into coordinated timeline");
                }
        auto rs = robots();
        rs[0].nominal_speed_mps = 1.0;
        rs[1].nominal_speed_mps = 1.5;
        rs[0].start = {"test", 5, 5};
        rs[1].start = {"test", 28, 5};
        const std::vector<MappedTask> tradeoff{make_mapped_task("farther-but-faster",
            {"test", 15, 5}, {"device"}, "inspection", 0)};
        const auto chosen = OfflineMissionPlanner(MultiMapPathPlanner(maps)).plan(rs, tradeoff, false);
        require(chosen.routes[0].stops.empty() && !chosen.routes[1].stops.empty(),
                "allocation chose shortest distance instead of speed-adjusted duration");
        rs[1].capabilities.clear();
        const auto restricted = OfflineMissionPlanner(MultiMapPathPlanner(maps)).plan(rs, tradeoff, false);
        require(!restricted.routes[0].stops.empty() && restricted.routes[1].stops.empty(),
                "speed advantage bypassed task capability");
        rs = robots();
        rs[0].nominal_speed_mps = 1.0;
        rs[1].nominal_speed_mps = 1.5;
        auto ts = tasks();
        AllocationOptions options;
        options.balance_task_counts = false;
        const auto timed = OfflineMissionPlanner(MultiMapPathPlanner(maps), {}, options).plan(rs, ts, false);
        options.balance_task_counts = true;
        const auto balanced = OfflineMissionPlanner(MultiMapPathPlanner(maps), {}, options).plan(rs, ts, false);
        verify(balanced, rs, ts, MultiMapPathPlanner(maps));
        const auto score = [&](const auto &plan)
        {
            std::pair<int, long long> load{0, 0};
            MultiMapPathPlanner planner(maps);
            for (std::size_t r = 0; r < rs.size(); ++r)
            {
                int ticks = plan.routes[r].service_ticks;
                auto previous = rs[r].start;
                for (const auto &stop : plan.routes[r].stops)
                {
                    ticks += planner.estimate_allocation_ticks(previous, stop.location,
                        rs[r].capabilities, 0.0, rs[r].nominal_speed_mps);
                    previous = stop.location;
                }
                load.first = std::max(load.first, ticks);
                load.second += ticks;
            }
            return load;
        };
        const auto before = score(timed), after = score(balanced);
        require(balanced.allocation_stats.balancing_moves > 0 &&
                after.first <= before.first * 1.1 + 1e-9 && after.second <= before.second * 1.1 + 1e-9,
                "final balancing guard ignored speed-adjusted load bound");
    }

    void invalid_options()
    {
        for (double value : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        {
            AllocationOptions options;
            options.budget_seconds = value;
            bool rejected = false;
            try { OfflineMissionPlanner p(MultiMapPathPlanner(bundle()), {}, options); }
            catch (const std::invalid_argument &) { rejected = true; }
            require(rejected, "invalid allocation budget accepted");
        }
        AllocationOptions options;
        options.balance_load_slack = -0.1;
        bool rejected = false;
        try { OfflineMissionPlanner p(MultiMapPathPlanner(bundle()), {}, options); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "negative allocation slack accepted");
        options = {};
        options.max_candidate_evaluations = 0;
        rejected = false;
        try { OfflineMissionPlanner p(MultiMapPathPlanner(bundle()), {}, options); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "zero candidate budget accepted");
    }

    void configuration()
    {
        const auto directory = std::filesystem::temp_directory_path() /
            ("allocation-config-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(directory);
        {
            std::ofstream pgm(directory / "test.pgm", std::ios::binary);
            pgm << "P5\n4 4\n255\n" << std::string(16, static_cast<char>(255));
            std::ofstream map(directory / "test.yaml");
            map << "image: test.pgm\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
        }
        auto root = YAML::Load("map: {directory: placeholder, persistent_cache: false}\nrobots: []\ntasks: []\n");
        root["map"]["directory"] = directory.string();
        root["planner"]["allocation"] = YAML::Load("{max_passes: 3, max_candidate_evaluations: 17, budget_seconds: 0.25, balance_task_counts: false, balance_load_slack: 0.05}");
        const auto loaded = OfflinePlannerConfigLoader::load_node(root, directory / "input.yaml");
        require(loaded.allocation.max_passes == 3 && loaded.allocation.max_candidate_evaluations == 17 &&
                loaded.allocation.budget_seconds == .25 && !loaded.allocation.balance_task_counts &&
                loaded.allocation.balance_load_slack == .05, "allocation configuration not forwarded");
        for (const auto *invalid : {"{budget_seconds: 0}", "{budget_seconds: .nan}", "{budget_seconds: .inf}",
                                  "{max_candidate_evaluations: 0}", "{balance_load_slack: -0.1}"})
        {
            root["planner"]["allocation"] = YAML::Load(invalid);
            bool rejected = false;
            try { (void)OfflinePlannerConfigLoader::load_node(root, directory / "input.yaml"); }
            catch (const std::runtime_error &) { rejected = true; }
            require(rejected, "invalid YAML allocation settings accepted");
        }
        std::filesystem::remove_all(directory);
    }
}

int main()
{
    try
    {
        large_allocation_and_load_bound();
        removal_return_and_budgets();
        allocation_speed_estimates();
        speed_aware_assignment();
        invalid_options();
        configuration();
        std::cout << "allocation tests passed\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
