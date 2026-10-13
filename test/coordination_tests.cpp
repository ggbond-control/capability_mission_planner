#include "../src/offline/coordination/multi_map_coordinator.hpp"
#include "../src/offline/coordination/safe_interval_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <queue>
#include <stdexcept>

using namespace capability_mission_planner::offline;

namespace
{
    void require(bool value, const char *message)
    {
        if (!value)
            throw std::runtime_error(message);
    }

    std::shared_ptr<MultiMapBundle> bundle()
    {
        auto result = std::make_shared<MultiMapBundle>();
        MapLayer map;
        map.id = "test";
        map.width = map.height = 100;
        map.resolution = 0.1;
        map.traversable.assign(10000, 1);
        map.clearance_m.assign(10000, 10.0F);
        map.inflated_cost.assign(10000, 0);
        result->maps.emplace(map.id, map);
        return result;
    }

    MappedRobot robot(const char *id, int x, int y, double radius = 0.0)
    {
        MappedRobot r;
        r.id = id;
        r.start = {"test", x, y};
        r.footprint_radius_m = radius;
        return r;
    }

    MultiMapPath path(std::initializer_list<PathStep> steps)
    {
        return {steps, (steps.end() - 1)->arrival_tick};
    }

    void verify(const MultiMapBundle &maps, const std::vector<MappedRobot> &robots,
                const std::vector<std::vector<TimedMapState>> &schedules)
    {
        require(schedules.size() == robots.size(), "missing robot schedules");
        std::size_t horizon = 0;
        for (std::size_t r = 0; r < robots.size(); ++r)
        {
            require(!schedules[r].empty() && schedules[r].front().position == robots[r].start,
                    "initial location changed");
            horizon = std::max(horizon, schedules[r].size());
            for (std::size_t t = 0; t < schedules[r].size(); ++t)
            {
                require(schedules[r][t].tick == static_cast<int>(t), "missing time step");
                if (t)
                    require(schedules[r][t].route_frame == schedules[r][t - 1].route_frame ||
                                schedules[r][t].route_frame == schedules[r][t - 1].route_frame + 1,
                            "route/service progress changed");
            }
        }
        const auto at = [&](std::size_t r, std::size_t t) -> const auto &
        { return schedules[r][std::min(t, schedules[r].size() - 1)]; };
        const auto pose = [&](const GridPosition &p)
        { const auto &map = maps.map(p.map_id); return map.local_to_root(map.grid_to_local(p)); };
        for (std::size_t t = 0; t < horizon; ++t)
            for (std::size_t a = 0; a < robots.size(); ++a)
                for (std::size_t b = a + 1; b < robots.size(); ++b)
                {
                    const auto &af = at(a, t), &bf = at(b, t);
                    const auto &an = at(a, t + 1), &bn = at(b, t + 1);
                    require(af.resource.empty() || af.resource != bf.resource, "resource conflict");
                    if (af.position.map_id != bf.position.map_id)
                        continue;
                    const double radius = robots[a].footprint_radius_m + robots[b].footprint_radius_m +
                                          robots[a].safety_margin_m + robots[b].safety_margin_m + 1e-9;
                    const auto p = pose(af.position), q = pose(bf.position);
                    const double dx = p.x - q.x, dy = p.y - q.y;
                    require(std::hypot(dx, dy) > radius, "vertex/footprint conflict");
                    if (af.position.map_id != an.position.map_id || bf.position.map_id != bn.position.map_id)
                        continue;
                    const auto pn = pose(an.position), qn = pose(bn.position);
                    const double vx = pn.x - p.x - (qn.x - q.x), vy = pn.y - p.y - (qn.y - q.y);
                    const double vv = vx * vx + vy * vy;
                    const double u = vv ? std::clamp(-(dx * vx + dy * vy) / vv, 0.0, 1.0) : 0.0;
                    require(std::hypot(dx + u * vx, dy + u * vy) > radius, "continuous swept conflict");
                }
    }

    void diagonal_edge_footprints()
    {
        auto maps = bundle();
        std::vector<MappedRobot> robots{robot("a", 10, 10, .3), robot("b", 22, 10, .3)};
        std::vector<MappedRobotRoute> routes(2);
        routes[0].segments = {path({{{"test", 10, 10}, 0, {}}, {{"test", 22, 22}, 2, {}}})};
        routes[1].segments = {path({{{"test", 22, 10}, 0, {}}, {{"test", 10, 22}, 2, {}}})};
        CoordinationStats stats;
        auto schedules = coordinate_multi_map_routes(MultiMapPathPlanner(maps), robots, routes, &stats);
        verify(*maps, robots, schedules);
        require(stats.total_wait_ticks > 0, "different edge resources must still enforce footprint safety");
    }

    void mandatory_service_and_cbs()
    {
        auto maps = bundle();
        std::vector<MappedRobot> robots{robot("a", 10, 20), robot("b", 20, 20)};
        std::vector<MappedRobotRoute> routes(2);
        routes[0].segments = {path({{{"test", 10, 20}, 0, {}}, {{"test", 20, 20}, 1, {}},
                                    {{"test", 30, 20}, 2, {}}})};
        routes[1].stops = {{{"test", 20, 20}, {}, 3, 0.0}};
        routes[1].segments = {path({{{"test", 20, 20}, 0, {}}}),
                              path({{{"test", 20, 20}, 0, {}}, {{"test", 20, 40}, 1, {}}})};
        CoordinationStats stats;
        auto schedules = coordinate_multi_map_routes(MultiMapPathPlanner(maps), robots, routes, &stats);
        verify(*maps, robots, schedules);
        require(stats.cbs_succeeded, "fixture must exercise CBS");
        require(schedules[1].back().route_frame == 4, "mandatory service frames lost");
    }

    void idle_goal_and_transitions()
    {
        auto maps = bundle();
        auto upper = maps->map("test");
        upper.id = "upper";
        maps->maps.emplace(upper.id, upper);
        std::vector<MappedRobot> robots{robot("a", 10, 20), robot("b", 30, 20), robot("idle", 90, 90)};
        std::vector<MappedRobotRoute> routes(3);
        routes[0].segments = {path({{{"test", 10, 20}, 0, {}}, {{"upper", 40, 40}, 8, "lift"},
                                    {{"upper", 60, 40}, 10, {}}})};
        routes[1].segments = {path({{{"test", 30, 20}, 0, {}}, {{"upper", 40, 40}, 8, "lift"},
                                    {{"upper", 40, 60}, 10, {}}})};
        CoordinationStats stats;
        auto schedules = coordinate_multi_map_routes(MultiMapPathPlanner(maps), robots, routes, &stats);
        verify(*maps, robots, schedules);
        require(schedules[2].size() == 1, "idle schedule changed");
        require(stats.total_wait_ticks > 0, "shared transition must insert waits");
    }

    void bounded_repair()
    {
        auto maps = bundle();
        std::vector<MappedRobot> robots{robot("short", 10, 20), robot("critical", 10, 40), robot("idle", 90, 90)};
        std::vector<MappedRobotRoute> routes(3);
        routes[0].stops = {{{"test", 20, 20}, {}, 40, 0.0}};
        routes[0].segments = {path({{{"test", 10, 20}, 0, {}}, {{"test", 20, 20}, 1, {}}}),
                              path({{{"test", 20, 20}, 0, {}}, {{"test", 30, 20}, 1, {}}})};
        routes[1].segments = {path({{{"test", 10, 40}, 0, {}}, {{"test", 20, 40}, 10, {}},
                                    {{"test", 30, 40}, 11, {}}, {{"test", 60, 40}, 90, {}}})};
        TraversalOptions options;
        options.shared_resources = {{"gate", {{"test", 20, 20}, {"test", 20, 40}}, 1, 0.0}};
        options.coordination_local_repair = false;
        CoordinationStats baseline, improved, exhausted;
        auto original = coordinate_multi_map_routes(MultiMapPathPlanner(maps, options), robots, routes, &baseline);
        options.coordination_local_repair = true;
        auto repaired = coordinate_multi_map_routes(MultiMapPathPlanner(maps, options), robots, routes, &improved);
        verify(*maps, robots, repaired);
        require(improved.local_repair_accepted && repaired[1].back().tick < original[1].back().tick,
                "bounded repair should shorten critical schedule");
        require(improved.total_wait_ticks <= baseline.total_wait_ticks, "repair increased total wait");
        require(repaired[0].back().route_frame == original[0].back().route_frame, "repair lost service");
        require(repaired[2].size() == original[2].size(), "repair changed frozen idle robot");
        const auto check_rollback = [&]()
        {
            auto rolled_back = coordinate_multi_map_routes(MultiMapPathPlanner(maps, options), robots, routes, &exhausted);
            require(exhausted.local_repair_attempted && exhausted.local_repair_budget_exhausted &&
                    !exhausted.local_repair_accepted, "exhausted repair must roll back");
            for (std::size_t r = 0; r < original.size(); ++r)
            {
                require(original[r].size() == rolled_back[r].size(), "rollback changed completion");
                for (std::size_t t = 0; t < original[r].size(); ++t)
                    require(original[r][t].route_frame == rolled_back[r][t].route_frame, "rollback changed trajectory");
            }
        };
        options.coordination_repair_max_expanded_nodes = 1;
        check_rollback();
        options.coordination_repair_max_expanded_nodes = 20000;
        options.coordination_repair_budget_seconds = 1e-12;
        check_rollback();
        options.coordination_repair_budget_seconds = .2;
        auto without_stats = coordinate_multi_map_routes(MultiMapPathPlanner(maps, options), robots, routes);
        verify(*maps, robots, without_stats);
        require(without_stats[1].back().tick == repaired[1].back().tick, "null stats changed repair behavior");
        // Promoting a short route can merely move waiting to a longer route.
        // This complete but worse candidate must also be discarded.
        routes[1].segments[0].steps.back().arrival_tick = 12;
        routes[1].segments[0].travel_ticks = 12;
        options.coordination_local_repair = false;
        auto before_rejected = coordinate_multi_map_routes(MultiMapPathPlanner(maps, options), robots, routes);
        options.coordination_local_repair = true;
        auto rejected = coordinate_multi_map_routes(MultiMapPathPlanner(maps, options), robots, routes, &exhausted);
        require(exhausted.local_repair_attempted && !exhausted.local_repair_accepted &&
                !exhausted.local_repair_budget_exhausted, "worse candidate must be discarded");
        for (std::size_t r = 0; r < rejected.size(); ++r)
            require(rejected[r].size() == before_rejected[r].size(), "rejected repair changed baseline");
    }

    // Independent tick BFS / geometry oracle. Random small routes include
    // service, forced transit, resource occupancy and permanently parked goals.
    void interval_search_matches_tick_oracle()
    {
        std::mt19937 random(1927);
        const int horizon = 24;
        for (int sample = 0; sample < 400; ++sample)
        {
            std::vector<MappedRobot> robots{robot("a", 0, 0, .1), robot("b", 0, 0, .1)};
            std::vector<std::vector<RouteFrame>> routes(2);
            for (auto &route : routes)
                for (int f = 0; f < 8; ++f)
                {
                    const int x = static_cast<int>(random() % 6), y = static_cast<int>(random() % 4);
                    std::string resource;
                    if (f > 0 && f < 7 && random() % 3 == 0) resource = "edge:shared";
                    else if (random() % 4 == 0) resource = "region:shared";
                    route.push_back({{"test", x, y}, resource, x * .5, y * .5});
                }
            std::vector<ReservationTrack> tracks{{1, {0, 1, 1, 2, 3, 4, 4, 5, 6, 7}}};
            const auto other = [&](int tick) -> const RouteFrame &
            { return routes[1][tracks[0].frames[std::min<std::size_t>(tick, tracks[0].frames.size() - 1)]]; };
            const auto occupied = [&](const RouteFrame &a, const RouteFrame &b)
            {
                return (!a.resource.empty() && a.resource == b.resource) ||
                    std::hypot(a.root_x - b.root_x, a.root_y - b.root_y) <= .2 + 1e-9;
            };
            const auto valid = [&](int frame, int next, int tick)
            {
                const auto &a = routes[0][frame], &an = routes[0][next], &b = other(tick), &bn = other(tick + 1);
                if (occupied(an, bn)) return false;
                double dx = a.root_x - b.root_x, dy = a.root_y - b.root_y;
                double vx = an.root_x - a.root_x - bn.root_x + b.root_x;
                double vy = an.root_y - a.root_y - bn.root_y + b.root_y;
                double vv = vx * vx + vy * vy;
                double u = vv > 1e-12 ? std::clamp(-(dx * vx + dy * vy) / vv, 0., 1.) : 0.;
                return std::hypot(dx + u * vx, dy + u * vy) > .2 + 1e-9;
            };
            int expected = -1;
            std::vector<std::vector<bool>> seen(horizon + 1, std::vector<bool>(8, false));
            std::queue<std::pair<int, int>> open;
            if (!occupied(routes[0][0], other(0))) { open.emplace(0, 0); seen[0][0] = true; }
            while (!open.empty())
            {
                auto [tick, frame] = open.front(); open.pop();
                if (frame == 7)
                {
                    bool parked_safe = true;
                    for (int t = tick; t < horizon; ++t)
                        if (!valid(frame, frame, t)) { parked_safe = false; break; }
                    if (parked_safe) { expected = tick; break; }
                }
                if (tick == horizon) continue;
                for (int next : {frame, frame + 1})
                {
                    if (next >= 8 || (next == frame && routes[0][frame].resource.compare(0, 5, "edge:") == 0)) continue;
                    if (!seen[tick + 1][next] && valid(frame, next, tick))
                    { seen[tick + 1][next] = true; open.emplace(tick + 1, next); }
                }
            }
            RouteReservations reservations(routes, robots, 0, tracks, horizon, nullptr);
            for (int t = 0; t < horizon; ++t)
                for (int f = 0; f < 8; ++f)
                {
                    require(reservations.allowed(f, f, t) == valid(f, f, t), "waiting reservation differs from geometry oracle");
                    if (f < 7) require(reservations.allowed(f, f + 1, t) == valid(f, f + 1, t), "advance reservation differs from geometry oracle");
                }
            std::vector<std::size_t> frames;
            const bool found = schedule_safe_intervals(reservations, routes[0], 0, frames, nullptr);
            require(found == (expected >= 0), "interval search feasibility differs from tick oracle");
            if (found)
            {
                require(static_cast<int>(frames.size()) - 1 == expected, "interval search lost earliest arrival");
                for (std::size_t t = 1; t < frames.size(); ++t)
                    require(valid(frames[t - 1], frames[t], static_cast<int>(t - 1)), "interval reconstruction unsafe");
            }
        }
    }

    void invalid_start_and_budget()
    {
        auto maps = bundle();
        std::vector<MappedRobot> robots{robot("a", 10, 10, .3), robot("b", 11, 10, .3)};
        bool rejected = false;
        try { coordinate_multi_map_routes(MultiMapPathPlanner(maps), robots, std::vector<MappedRobotRoute>(2)); }
        catch (const std::runtime_error &) { rejected = true; }
        require(rejected, "overlapping initial footprints must be rejected");
        for (double budget : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        {
            TraversalOptions options;
            options.coordination_repair_budget_seconds = budget;
            rejected = false;
            try { MultiMapPathPlanner planner(maps, options); }
            catch (const std::invalid_argument &) { rejected = true; }
            require(rejected, "invalid repair budget accepted");
        }
    }
}

int main()
{
    try
    {
        diagonal_edge_footprints();
        mandatory_service_and_cbs();
        idle_goal_and_transitions();
        bounded_repair();
        interval_search_matches_tick_oracle();
        invalid_start_and_budget();
        std::cout << "coordination tests passed\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
