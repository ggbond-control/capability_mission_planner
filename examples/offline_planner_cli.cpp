#include <capability_mission_planner/offline_planner_config.hpp>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <sys/resource.h>

using namespace capability_mission_planner::offline;

int main(int argc, char *argv[])
{
    try
    {
        if (argc < 2 || argc > 3)
        {
            std::cerr << "usage: capability_mission_planner_cli CONFIG.yaml [OUTPUT_DIRECTORY]\n";
            return 2;
        }
        const auto total_start = std::chrono::steady_clock::now();
        auto request = OfflinePlannerConfigLoader::load(argv[1]);
        const auto loaded_at = std::chrono::steady_clock::now();
        if (argc == 3)
            request.output_directory = std::filesystem::absolute(argv[2]);

        MultiMapPathPlanner path_planner(request.bundle, request.traversal);
        OfflineMissionPlanner planner(std::move(path_planner), request.objective);
        const auto plan = planner.plan(request.robots, request.tasks, request.coordinate_conflicts);
        const auto planned_at = std::chrono::steady_clock::now();
        PlanExporter::write(request.output_directory, *request.bundle, request.robots, request.tasks, plan, request.export_options);
        const auto exported_at = std::chrono::steady_clock::now();
        const auto seconds = [](const auto begin, const auto end)
        {
            return std::chrono::duration<double>(end - begin).count();
        };
        struct rusage usage{};
        getrusage(RUSAGE_SELF, &usage);

        std::cout << "planning complete\n"
                  << "  maps: " << request.bundle->maps.size() << '\n'
                  << "  map_cache_hits: " << request.bundle->map_cache_hits << '\n'
                  << "  map_cache_misses: " << request.bundle->map_cache_misses << '\n'
                  << "  map_cache_validation_seconds: "
                  << request.bundle->map_cache_validation_seconds << '\n'
                  << "  map_cache_read_seconds: "
                  << request.bundle->map_cache_read_seconds << '\n'
                  << "  map_preprocess_seconds: "
                  << request.bundle->map_preprocess_seconds << '\n'
                  << "  robots: " << request.robots.size() << '\n'
                  << "  tasks: " << request.tasks.size() << '\n'
                  << "  maximum_load_seconds: "
                  << plan.maximum_load_ticks * plan.time_step_seconds << '\n'
                  << "  total_load_seconds: "
                  << plan.total_load_ticks * plan.time_step_seconds << '\n'
                  << "  allocation_estimate_requests: "
                  << plan.allocation_path_stats.estimate_requests << '\n'
                  << "  allocation_grid_searches: "
                  << plan.allocation_path_stats.grid_searches << '\n'
                  << "  allocation_a_star_searches: "
                  << plan.allocation_path_stats.a_star_searches << '\n'
                  << "  allocation_expanded_nodes: "
                  << plan.allocation_path_stats.expanded_nodes << '\n'
                  << "  allocation_cache_hits: "
                  << plan.allocation_path_stats.cache_hits << '\n'
                  << "  allocation_cache_hit_ratio: "
                  << (plan.allocation_path_stats.estimate_requests == 0U ? 0.0 : static_cast<double>(plan.allocation_path_stats.cache_hits) / static_cast<double>(plan.allocation_path_stats.estimate_requests)) << '\n'
                  << "  allocation_grid_search_seconds: "
                  << plan.allocation_path_stats.grid_search_seconds << '\n'
                  << "  allocation_distance_field_searches: "
                  << plan.allocation_path_stats.distance_field_searches << '\n'
                  << "  allocation_distance_field_seconds: "
                  << plan.allocation_path_stats.distance_field_seconds << '\n'
                  << "  total_grid_searches: "
                  << plan.total_path_stats.grid_searches << '\n'
                  << "  total_a_star_searches: "
                  << plan.total_path_stats.a_star_searches << '\n'
                  << "  total_expanded_nodes: "
                  << plan.total_path_stats.expanded_nodes << '\n'
                  << "  total_cache_hits: "
                  << plan.total_path_stats.cache_hits << '\n'
                  << "  total_grid_search_seconds: "
                  << plan.total_path_stats.grid_search_seconds << '\n'
                  << "  total_distance_field_searches: "
                  << plan.total_path_stats.distance_field_searches << '\n'
                  << "  total_distance_field_seconds: "
                  << plan.total_path_stats.distance_field_seconds << '\n'
                  << "  timing_allocation_seconds: "
                  << plan.allocation_seconds << '\n'
                  << "  timing_estimate_precompute_seconds: "
                  << plan.estimate_precompute_seconds << '\n'
                  << "  timing_final_path_seconds: "
                  << plan.final_path_seconds << '\n'
                  << "  timing_coordination_seconds: "
                  << plan.coordination_seconds << '\n'
                  << "  coordination_prioritized_succeeded: "
                  << (plan.coordination_stats.prioritized_succeeded ? "true" : "false") << '\n'
                  << "  coordination_prioritized_low_level_searches: "
                  << plan.coordination_stats.prioritized_low_level_searches << '\n'
                  << "  coordination_prioritized_expanded_nodes: "
                  << plan.coordination_stats.prioritized_low_level_expanded_nodes << '\n'
                  << "  coordination_cbs_started: "
                  << (plan.coordination_stats.cbs_started ? "true" : "false") << '\n'
                  << "  coordination_cbs_high_level_expanded_nodes: "
                  << plan.coordination_stats.cbs_high_level_expanded_nodes << '\n'
                  << "  coordination_cbs_low_level_searches: "
                  << plan.coordination_stats.cbs_low_level_searches << '\n'
                  << "  coordination_cbs_low_level_expanded_nodes: "
                  << plan.coordination_stats.cbs_low_level_expanded_nodes << '\n'
                  << "  coordination_conflict_checks: "
                  << plan.coordination_stats.conflict_checks << '\n'
                  << "  coordination_conflict_check_seconds: "
                  << plan.coordination_stats.conflict_check_seconds << '\n'
                  << "  coordination_total_wait_seconds: "
                  << plan.coordination_stats.total_wait_ticks * plan.time_step_seconds << '\n';
        for (std::size_t i = 0; i < plan.coordination_stats.route_frames_by_robot.size(); ++i)
        {
            const auto &frame_expansions = plan.coordination_stats.prioritized_frame_expansions_by_robot[i];
            const auto hottest = std::max_element(frame_expansions.begin(), frame_expansions.end());
            std::cout << "  coordination_robot_" << i << "_route_frames: "
                      << plan.coordination_stats.route_frames_by_robot[i] << '\n'
                      << "  coordination_robot_" << i << "_prioritized_searches: "
                      << plan.coordination_stats.prioritized_searches_by_robot[i] << '\n'
                      << "  coordination_robot_" << i << "_prioritized_expanded_nodes: "
                      << plan.coordination_stats.prioritized_expanded_nodes_by_robot[i] << '\n'
                      << "  coordination_robot_" << i << "_hottest_frame: "
                      << (hottest == frame_expansions.end() ? 0U : static_cast<std::size_t>(std::distance(frame_expansions.begin(), hottest))) << '\n'
                      << "  coordination_robot_" << i << "_hottest_frame_expansions: "
                      << (hottest == frame_expansions.end() ? 0U : *hottest) << '\n';
        }
        std::cout << "  peak_rss_mb: "
                  << static_cast<double>(usage.ru_maxrss) / 1024.0 << '\n'
                  << "  timing_load_and_parse_seconds: "
                  << seconds(total_start, loaded_at) << '\n'
                  << "  timing_planning_seconds: "
                  << seconds(loaded_at, planned_at) << '\n'
                  << "  timing_export_seconds: "
                  << seconds(planned_at, exported_at) << '\n'
                  << "  timing_total_seconds: "
                  << seconds(total_start, exported_at) << '\n'
                  << "  output: " << request.output_directory << '\n';
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "planning failed: " << error.what() << '\n';
        return 1;
    }
}
