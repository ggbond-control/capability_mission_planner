#pragma once

#include <capability_mission_planner/offline/offline_map_planner.hpp>

#include <functional>
#include <cstdint>
#include <map>
#include <optional>
#include <unordered_map>

namespace capability_mission_planner::offline
{
    struct RouteFrame
    {
        GridPosition position;
        std::string resource;
        double root_x = 0.0;
        double root_y = 0.0;
    };

    struct TickInterval
    {
        int begin;
        int end; // inclusive
    };

    struct ReservationTrack
    {
        std::size_t robot;
        std::vector<std::size_t> frames;
    };

    // Each frozen trajectory is compressed into constant occupancy/motion runs.
    // The geometry is identical to the tick-level coordinator, including its
    // explicit transit resources. Final positions remain reserved to the horizon.
    class RouteReservations
    {
    public:
        RouteReservations(const std::vector<std::vector<RouteFrame>> &routes,
                          const std::vector<MappedRobot> &robots, std::size_t agent,
                          const std::vector<ReservationTrack> &tracks, int horizon,
                          CoordinationStats *stats,
                          std::function<void()> check_budget = {});

        bool allowed(std::size_t from, std::size_t to, int departure);
        const std::vector<TickInterval> &safe_intervals(std::size_t frame);
        const std::vector<TickInterval> &blocked_advance(std::size_t frame);
        void record_blocked_departures(std::size_t frame, int begin, int end);
        int horizon() const { return _horizon; }
        void check_budget() const;

        static std::vector<TickInterval> merge(std::vector<TickInterval> intervals);
        static bool contains(const std::vector<TickInterval> &intervals, int tick);

    private:
        struct Run
        {
            std::size_t robot;
            const RouteFrame *from;
            const RouteFrame *to;
            int begin;
            int end;
        };
        struct Cache
        {
            std::vector<std::pair<TickInterval, std::size_t>> occupancy_owners;
            std::vector<std::pair<TickInterval, std::size_t>> advance_owners;
            std::optional<std::vector<TickInterval>> occupancy;
            std::optional<std::vector<TickInterval>> advance;
            std::optional<std::vector<TickInterval>> waiting;
            std::optional<std::vector<TickInterval>> safe;
        };
        using Index = std::map<std::string, std::unordered_map<std::int64_t, std::vector<std::size_t>>>;

        const std::vector<std::vector<RouteFrame>> &_routes;
        const std::vector<MappedRobot> &_robots;
        std::size_t _agent;
        int _horizon;
        double _bucket_size;
        CoordinationStats *_stats;
        std::function<void()> _check_budget;
        std::vector<Run> _occupancy;
        std::vector<Run> _motion;
        Index _occupancy_index;
        Index _motion_index;
        std::unordered_map<std::string, std::vector<std::size_t>> _resources;
        std::unordered_map<std::size_t, Cache> _cache;

        double radius(std::size_t other) const;
        void index_run(Index &index, const Run &run, std::size_t id, bool motion);
        std::vector<std::size_t> nearby(const Index &index, const RouteFrame &from,
                                        const RouteFrame &to, double padding) const;
        const std::vector<TickInterval> &blocked_occupancy(std::size_t frame);
        const std::vector<TickInterval> &blocked_wait(std::size_t frame);
        std::vector<TickInterval> blocked_motion(std::size_t from, std::size_t to);
    };
}
