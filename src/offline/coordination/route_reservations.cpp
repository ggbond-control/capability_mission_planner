#include "route_reservations.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace capability_mission_planner::offline
{
    namespace
    {
        bool same_position(const RouteFrame &a, const RouteFrame &b)
        {
            return a.position == b.position;
        }

        std::int64_t key(int x, int y)
        {
            return static_cast<std::int64_t>((static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
                                             static_cast<std::uint32_t>(y));
        }

        bool near(const RouteFrame &a, const RouteFrame &b, double radius)
        {
            if (a.position.map_id != b.position.map_id)
                return false;
            const double dx = a.root_x - b.root_x, dy = a.root_y - b.root_y;
            return dx * dx + dy * dy <= radius * radius;
        }

        bool swept(const RouteFrame &af, const RouteFrame &at,
                   const RouteFrame &bf, const RouteFrame &bt, double radius)
        {
            if (af.position.map_id != at.position.map_id || bf.position.map_id != bt.position.map_id ||
                af.position.map_id != bf.position.map_id)
                return false;
            const double dx = af.root_x - bf.root_x, dy = af.root_y - bf.root_y;
            const double vx = at.root_x - af.root_x - (bt.root_x - bf.root_x);
            const double vy = at.root_y - af.root_y - (bt.root_y - bf.root_y);
            const double vv = vx * vx + vy * vy;
            const double u = vv > 1e-12 ? std::clamp(-(dx * vx + dy * vy) / vv, 0.0, 1.0) : 0.0;
            const double x = dx + u * vx, y = dy + u * vy;
            return x * x + y * y <= radius * radius;
        }
    }

    RouteReservations::RouteReservations(const std::vector<std::vector<RouteFrame>> &routes,
                                         const std::vector<MappedRobot> &robots, std::size_t agent,
                                         const std::vector<ReservationTrack> &tracks, int horizon,
                                         CoordinationStats *stats, std::function<void()> check_budget)
        : _routes(routes), _robots(robots), _agent(agent), _horizon(horizon), _bucket_size(1.0),
          _stats(stats), _check_budget(std::move(check_budget))
    {
        for (const auto &robot : robots)
            _bucket_size = std::max(_bucket_size, 2 * (robot.footprint_radius_m + robot.safety_margin_m) + 1e-8);
        for (const auto &track : tracks)
        {
            this->check_budget();
            if (track.frames.empty())
                throw std::invalid_argument("cannot reserve an empty trajectory");
            const auto &route = routes.at(track.robot);
            const auto first_occupancy = _occupancy.size();
            const auto first_motion = _motion.size();
            for (std::size_t t = 0; t < track.frames.size(); ++t)
            {
                if ((t & 255U) == 0U)
                    this->check_budget();
                const auto &frame = route.at(track.frames[t]);
                const int tick = static_cast<int>(t);
                if (_occupancy.size() > first_occupancy &&
                    same_position(*_occupancy.back().from, frame) && _occupancy.back().from->resource == frame.resource)
                    _occupancy.back().end = tick;
                else
                    _occupancy.push_back({track.robot, &frame, &frame, tick, tick});
                const auto &next = route.at(track.frames[std::min(t + 1U, track.frames.size() - 1U)]);
                if (_motion.size() > first_motion && same_position(*_motion.back().from, frame) &&
                    same_position(*_motion.back().to, next))
                    _motion.back().end = tick;
                else
                    _motion.push_back({track.robot, &frame, &next, tick, tick});
            }
            _occupancy.back().end = horizon;
            _motion.back().end = horizon - 1;
        }
        for (std::size_t i = 0; i < _occupancy.size(); ++i)
        {
            if ((i & 255U) == 0U) this->check_budget();
            index_run(_occupancy_index, _occupancy[i], i, false);
            if (!_occupancy[i].from->resource.empty())
                _resources[_occupancy[i].from->resource].push_back(i);
        }
        for (std::size_t i = 0; i < _motion.size(); ++i)
        {
            if ((i & 255U) == 0U) this->check_budget();
            index_run(_motion_index, _motion[i], i, true);
        }
        if (_stats)
        {
            _stats->reservation_intervals += _occupancy.size() + _motion.size();
            if (_stats->blocking_departure_ticks_by_robot.empty())
                _stats->blocking_departure_ticks_by_robot.assign(robots.size(), std::vector<std::size_t>(robots.size(), 0U));
        }
    }

    void RouteReservations::check_budget() const
    {
        if (_check_budget)
            _check_budget();
    }

    double RouteReservations::radius(std::size_t other) const
    {
        return _robots[_agent].footprint_radius_m + _robots[other].footprint_radius_m +
               _robots[_agent].safety_margin_m + _robots[other].safety_margin_m + 1e-9;
    }

    void RouteReservations::index_run(Index &index, const Run &run, std::size_t id, bool motion)
    {
        if (motion && run.from->position.map_id != run.to->position.map_id)
            return;
        const auto &a = *run.from, &b = *run.to;
        const int x0 = static_cast<int>(std::floor(std::min(a.root_x, b.root_x) / _bucket_size));
        const int x1 = static_cast<int>(std::floor(std::max(a.root_x, b.root_x) / _bucket_size));
        const int y0 = static_cast<int>(std::floor(std::min(a.root_y, b.root_y) / _bucket_size));
        const int y1 = static_cast<int>(std::floor(std::max(a.root_y, b.root_y) / _bucket_size));
        auto &buckets = index[a.position.map_id];
        for (int x = x0; x <= x1; ++x)
        {
            check_budget();
            for (int y = y0; y <= y1; ++y)
                buckets[key(x, y)].push_back(id);
        }
    }

    std::vector<std::size_t> RouteReservations::nearby(const Index &index, const RouteFrame &from,
                                                       const RouteFrame &to, double padding) const
    {
        std::vector<std::size_t> ids;
        const auto map = index.find(from.position.map_id);
        if (map == index.end())
            return ids;
        const int x0 = static_cast<int>(std::floor((std::min(from.root_x, to.root_x) - padding) / _bucket_size));
        const int x1 = static_cast<int>(std::floor((std::max(from.root_x, to.root_x) + padding) / _bucket_size));
        const int y0 = static_cast<int>(std::floor((std::min(from.root_y, to.root_y) - padding) / _bucket_size));
        const int y1 = static_cast<int>(std::floor((std::max(from.root_y, to.root_y) + padding) / _bucket_size));
        for (int x = x0; x <= x1; ++x)
        {
            check_budget();
            for (int y = y0; y <= y1; ++y)
            {
                const auto bucket = map->second.find(key(x, y));
                if (bucket != map->second.end())
                    ids.insert(ids.end(), bucket->second.begin(), bucket->second.end());
            }
        }
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
        return ids;
    }

    std::vector<TickInterval> RouteReservations::merge(std::vector<TickInterval> intervals)
    {
        std::sort(intervals.begin(), intervals.end(), [](const auto &a, const auto &b)
                  { return a.begin < b.begin || (a.begin == b.begin && a.end < b.end); });
        std::vector<TickInterval> result;
        for (const auto &interval : intervals)
        {
            if (interval.end < interval.begin)
                continue;
            if (!result.empty() && interval.begin <= result.back().end + 1)
                result.back().end = std::max(result.back().end, interval.end);
            else
                result.push_back(interval);
        }
        return result;
    }

    bool RouteReservations::contains(const std::vector<TickInterval> &intervals, int tick)
    {
        const auto next = std::upper_bound(intervals.begin(), intervals.end(), tick,
                                          [](int value, const auto &interval) { return value < interval.begin; });
        return next != intervals.begin() && std::prev(next)->end >= tick;
    }

    const std::vector<TickInterval> &RouteReservations::blocked_occupancy(std::size_t frame)
    {
        auto &cached = _cache[frame].occupancy;
        if (cached)
            return *cached;
        check_budget();
        const auto &point = _routes[_agent][frame];
        double padding = 0;
        for (std::size_t other = 0; other < _robots.size(); ++other)
            padding = std::max(padding, radius(other));
        auto ids = nearby(_occupancy_index, point, point, padding);
        if (!point.resource.empty())
        {
            const auto resource = _resources.find(point.resource);
            if (resource != _resources.end())
                ids.insert(ids.end(), resource->second.begin(), resource->second.end());
        }
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
        std::vector<TickInterval> result;
        for (const auto id : ids)
        {
            if ((id & 255U) == 0U) check_budget();
            const auto &run = _occupancy[id];
            if (near(point, *run.from, radius(run.robot)) ||
                (!point.resource.empty() && point.resource == run.from->resource))
            {
                result.push_back({run.begin, run.end});
                _cache[frame].occupancy_owners.push_back({{run.begin, run.end}, run.robot});
            }
        }
        cached = merge(std::move(result));
        return *cached;
    }

    std::vector<TickInterval> RouteReservations::blocked_motion(std::size_t from, std::size_t to)
    {
        check_budget();
        const auto &a = _routes[_agent][from], &b = _routes[_agent][to];
        std::vector<TickInterval> result;
        // Occupancy is evaluated at arrival, so shift those intervals one tick.
        for (const auto &interval : blocked_occupancy(to))
            result.push_back({std::max(0, interval.begin - 1), interval.end - 1});
        if (from != to)
            for (const auto &[interval, owner] : _cache[to].occupancy_owners)
                _cache[from].advance_owners.push_back({{std::max(0, interval.begin - 1), interval.end - 1}, owner});
        if (a.position.map_id == b.position.map_id)
        {
            double padding = 0;
            for (std::size_t other = 0; other < _robots.size(); ++other)
                padding = std::max(padding, radius(other));
            for (const auto id : nearby(_motion_index, a, b, padding))
            {
                if ((id & 255U) == 0U) check_budget();
                const auto &run = _motion[id];
                if (swept(a, b, *run.from, *run.to, radius(run.robot)))
                {
                    result.push_back({run.begin, run.end});
                    if (from != to)
                        _cache[from].advance_owners.push_back({{run.begin, run.end}, run.robot});
                }
            }
        }
        return merge(std::move(result));
    }

    const std::vector<TickInterval> &RouteReservations::blocked_advance(std::size_t frame)
    {
        if (_stats) ++_stats->reservation_queries;
        auto &cached = _cache[frame].advance;
        if (!cached)
            cached = blocked_motion(frame, frame + 1);
        return *cached;
    }

    const std::vector<TickInterval> &RouteReservations::blocked_wait(std::size_t frame)
    {
        auto &cached = _cache[frame].waiting;
        if (!cached)
            cached = blocked_motion(frame, frame);
        return *cached;
    }

    bool RouteReservations::allowed(std::size_t from, std::size_t to, int departure)
    {
        if (_stats)
            ++_stats->reservation_queries;
        const bool result = !contains(from == to ? blocked_wait(from) : blocked_advance(from), departure);
        if (!result && from != to)
            record_blocked_departures(from, departure, departure);
        return result;
    }

    void RouteReservations::record_blocked_departures(std::size_t frame, int begin, int end)
    {
        if (!_stats || end < begin)
            return;
        (void)blocked_advance(frame);
        // Union each owner's intervals to avoid counting occupancy and swept
        // rejection twice for the same departure and blocker.
        std::map<std::size_t, std::vector<TickInterval>> owners;
        for (const auto &[interval, owner] : _cache[frame].advance_owners)
            if (interval.begin <= end && interval.end >= begin)
                owners[owner].push_back({std::max(begin, interval.begin), std::min(end, interval.end)});
        for (auto &[owner, intervals] : owners)
            for (const auto &interval : merge(std::move(intervals)))
                _stats->blocking_departure_ticks_by_robot[_agent][owner] +=
                    static_cast<std::size_t>(interval.end - interval.begin + 1);
    }

    const std::vector<TickInterval> &RouteReservations::safe_intervals(std::size_t frame)
    {
        if (_stats) ++_stats->reservation_queries;
        auto &cached = _cache[frame].safe;
        if (cached)
            return *cached;
        std::vector<TickInterval> safe;
        int begin = 0;
        for (const auto &blocked : blocked_occupancy(frame))
        {
            if (begin < blocked.begin)
                safe.push_back({begin, blocked.begin - 1});
            begin = std::max(begin, blocked.end + 1);
        }
        if (begin <= _horizon)
            safe.push_back({begin, _horizon});
        // Even two safe endpoint ticks cannot be connected by waiting when a
        // moving footprint crosses the stationary robot between those ticks.
        std::vector<TickInterval> split;
        const auto &waiting = blocked_wait(frame);
        for (const auto &interval : safe)
        {
            check_budget();
            int start = interval.begin;
            for (const auto &blocked : waiting)
            {
                if (blocked.end < start || blocked.begin >= interval.end)
                    continue;
                const int lo = std::max(start, blocked.begin);
                const int hi = std::min(interval.end - 1, blocked.end);
                split.push_back({start, lo});
                for (int t = lo + 1; t <= hi; ++t)
                {
                    if ((t & 255) == 0) check_budget();
                    split.push_back({t, t});
                }
                start = hi + 1;
            }
            if (start <= interval.end)
                split.push_back({start, interval.end});
        }
        cached = std::move(split);
        return *cached;
    }
}
