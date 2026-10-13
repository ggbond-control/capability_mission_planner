#include <capability_mission_planner/offline/offline_planner_config.hpp>
#include "../src/offline/coordination/multi_map_coordinator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sys/resource.h>

using namespace capability_mission_planner::offline;

// Measures coordination only, after allocation / static path planning. Checks
// safety independently in ROOT coordinates, including the parked suffix.
int main(int argc, char **argv)
{
    try
    {
        if (argc < 2)
            throw std::invalid_argument("usage: coordination_benchmark CONFIG [--no-repair] [--rotate=N] [--csv=PATH]");
        auto request = OfflinePlannerConfigLoader::load(argv[1]);
        std::size_t rotate = 0;
        std::string csv_path;
        for (int i = 2; i < argc; ++i)
        {
            const std::string arg(argv[i]);
            if (arg == "--no-repair") request.traversal.coordination_local_repair = false;
            else if (arg.compare(0, 9, "--rotate=") == 0) rotate = std::stoul(arg.substr(9));
            else if (arg.compare(0, 6, "--csv=") == 0) csv_path = arg.substr(6);
            else throw std::invalid_argument("unknown benchmark argument: " + arg);
        }
        MultiMapPathPlanner paths(request.bundle, request.traversal);
        auto plan = OfflineMissionPlanner(paths, request.objective).plan(request.robots, request.tasks, false);
        // Vary coordination order without changing the allocation or routes.
        if (!request.robots.empty())
        {
            rotate %= request.robots.size();
            std::rotate(request.robots.begin(), request.robots.begin() + rotate, request.robots.end());
            std::rotate(plan.routes.begin(), plan.routes.begin() + rotate, plan.routes.end());
        }
        CoordinationStats stats;
        const auto start = std::chrono::steady_clock::now();
        const auto schedules = coordinate_multi_map_routes(paths, request.robots, plan.routes, &stats);
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        int horizon = 0;
        long long total = 0;
        for (const auto &schedule : schedules)
        {
            horizon = std::max(horizon, schedule.back().tick);
            total += schedule.back().tick;
        }
        const auto at = [&](std::size_t robot, int tick) -> const TimedMapState &
        { return schedules[robot][std::min<std::size_t>(tick, schedules[robot].size() - 1U)]; };
        const auto pose = [&](const GridPosition &p)
        { const auto &map = request.bundle->map(p.map_id); return map.local_to_root(map.grid_to_local(p)); };
        std::size_t unsafe = 0;
        for (int t = 0; t <= horizon; ++t)
            for (std::size_t a = 0; a < schedules.size(); ++a)
                for (std::size_t b = a + 1; b < schedules.size(); ++b)
                {
                    const auto &af = at(a, t), &bf = at(b, t), &an = at(a, t + 1), &bn = at(b, t + 1);
                    if (!af.resource.empty() && af.resource == bf.resource) { ++unsafe; continue; }
                    if (af.position.map_id != bf.position.map_id) continue;
                    const auto p = pose(af.position), q = pose(bf.position);
                    const double dx = p.x - q.x, dy = p.y - q.y;
                    const auto &ra = request.robots[a], &rb = request.robots[b];
                    const double radius = ra.footprint_radius_m + rb.footprint_radius_m +
                                          ra.safety_margin_m + rb.safety_margin_m + 1e-9;
                    if (std::hypot(dx, dy) <= radius) { ++unsafe; continue; }
                    if (af.position.map_id != an.position.map_id || bf.position.map_id != bn.position.map_id) continue;
                    const auto pn = pose(an.position), qn = pose(bn.position);
                    const double vx = pn.x - p.x - qn.x + q.x, vy = pn.y - p.y - qn.y + q.y;
                    const double vv = vx * vx + vy * vy;
                    const double u = vv > 1e-12 ? std::clamp(-(dx * vx + dy * vy) / vv, 0., 1.) : 0.;
                    if (std::hypot(dx + u * vx, dy + u * vy) <= radius) ++unsafe;
                }
        if (!csv_path.empty())
        {
            std::ofstream csv(csv_path);
            if (!csv) throw std::runtime_error("cannot write benchmark CSV");
            csv << "robot,tick,map,x,y,frame,resource\n";
            for (std::size_t r = 0; r < schedules.size(); ++r)
                for (const auto &state : schedules[r])
                    csv << request.robots[r].id << ',' << state.tick << ',' << state.position.map_id << ','
                        << state.position.x << ',' << state.position.y << ',' << state.route_frame << ',' << state.resource << '\n';
        }
        rusage usage{};
        getrusage(RUSAGE_SELF, &usage);
        std::cout << "{\"seconds\":" << seconds << ",\"rss_mb\":" << usage.ru_maxrss / 1024.
                  << ",\"makespan\":" << horizon * request.traversal.time_step_seconds
                  << ",\"total_finish\":" << total * request.traversal.time_step_seconds
                  << ",\"wait\":" << stats.total_wait_ticks * request.traversal.time_step_seconds
                  << ",\"expanded\":" << stats.prioritized_low_level_expanded_nodes
                  << ",\"cbs\":" << stats.cbs_started << ",\"unsafe\":" << unsafe
                  << ",\"repair_attempted\":" << stats.local_repair_attempted
                  << ",\"repair_accepted\":" << stats.local_repair_accepted
                  << ",\"repair_budget_exhausted\":" << stats.local_repair_budget_exhausted
                  << ",\"repair_seconds\":" << stats.local_repair_seconds
                  << ",\"repair_expanded\":" << stats.local_repair_expanded_nodes << ",\"repair_robots\": [";
        for (std::size_t i = 0; i < stats.local_repair_robots.size(); ++i)
        {
            if (i) std::cout << ',';
            std::cout << stats.local_repair_robots[i];
        }
        std::cout << "]}\n";
        return unsafe ? 3 : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
