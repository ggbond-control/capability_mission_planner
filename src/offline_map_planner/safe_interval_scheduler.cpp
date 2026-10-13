#include "safe_interval_scheduler.hpp"

#include <algorithm>
#include <map>
#include <queue>
#include <tuple>

namespace capability_mission_planner::offline
{
    namespace
    {
        bool transit(const RouteFrame &frame)
        {
            return frame.resource.compare(0, 5, "edge:") == 0 ||
                   frame.resource.compare(0, 11, "transition:") == 0;
        }

        struct Node
        {
            std::size_t frame;
            std::size_t interval;
            int arrival;
            std::size_t parent;
            int departure;
        };

        struct MacroEdge
        {
            std::size_t next;
            int duration;
            std::vector<TickInterval> blocked;
        };
    }

    bool schedule_safe_intervals(RouteReservations &reservations, const std::vector<RouteFrame> &route,
                                 std::size_t robot, std::vector<std::size_t> &frames,
                                 CoordinationStats *stats, std::size_t node_limit)
    {
        frames.clear();
        const auto &start_intervals = reservations.safe_intervals(0);
        auto start = std::find_if(start_intervals.begin(), start_intervals.end(),
                                  [](const auto &interval) { return interval.begin == 0; });
        if (start == start_intervals.end())
            return false;
        const std::size_t initial_interval = std::distance(start_intervals.begin(), start);
        // Earliest arrival dominates later arrivals only within one interval
        // where waiting is safe. No states are merged across interval boundaries.
        std::map<std::pair<std::size_t, std::size_t>, int> best;
        using Entry = std::tuple<int, int, std::size_t>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        std::vector<Node> nodes;
        nodes.push_back({0, initial_interval, 0, 0, 0});
        best[{0, initial_interval}] = 0;
        open.emplace(static_cast<int>(route.size() - 1U), 0, 0);
        std::map<std::size_t, MacroEdge> edges;
        std::size_t expanded = 0;
        while (!open.empty())
        {
            reservations.check_budget();
            const auto id = std::get<2>(open.top());
            open.pop();
            const auto node = nodes[id];
            if (best[{node.frame, node.interval}] != node.arrival)
                continue;
            if (node_limit && expanded >= node_limit)
                return false;
            ++expanded;
            if (stats)
            {
                ++stats->safe_interval_expanded_nodes;
                ++stats->prioritized_low_level_expanded_nodes;
                ++stats->prioritized_expanded_nodes_by_robot[robot];
                auto &expansions = stats->prioritized_frame_expansions_by_robot[robot];
                if (expansions.size() <= node.frame)
                    expansions.resize(node.frame + 1, 0);
                ++expansions[node.frame];
            }
            const auto &interval = reservations.safe_intervals(node.frame)[node.interval];
            if (node.frame + 1U == route.size())
            {
                // A finished robot remains parked. A temporarily safe goal is
                // insufficient if a frozen trajectory visits it later.
                if (interval.end != reservations.horizon())
                    continue;
                std::vector<std::size_t> chain;
                for (std::size_t current = id; current; current = nodes[current].parent)
                    chain.push_back(current);
                std::reverse(chain.begin(), chain.end());
                frames.push_back(0);
                std::size_t previous_frame = 0;
                for (const auto current : chain)
                {
                    reservations.check_budget();
                    const auto &step = nodes[current];
                    const auto &parent = nodes[step.parent];
                    if (stats && step.departure > parent.arrival)
                        for (auto f = parent.frame; f < step.frame; ++f)
                        {
                            const int offset = static_cast<int>(f - parent.frame);
                            reservations.record_blocked_departures(f, parent.arrival + offset,
                                                                    step.departure - 1 + offset);
                        }
                    while (static_cast<int>(frames.size()) - 1 < step.departure)
                    {
                        if ((frames.size() & 255U) == 0U) reservations.check_budget();
                        frames.push_back(previous_frame);
                    }
                    for (auto f = previous_frame + 1; f <= step.frame; ++f)
                        frames.push_back(f);
                    previous_frame = step.frame;
                }
                return true;
            }
            if (!edges.count(node.frame))
            {
                std::size_t next = node.frame + 1U;
                while (next + 1U < route.size() && transit(route[next]))
                    ++next;
                MacroEdge edge{next, static_cast<int>(next - node.frame), {}};
                // A transit cannot pause midway. Intersect all per-tick motion
                // constraints by shifting them to the macro's departure time.
                for (std::size_t f = node.frame; f < next; ++f)
                {
                    reservations.check_budget();
                    const int offset = static_cast<int>(f - node.frame);
                    for (const auto &blocked : reservations.blocked_advance(f))
                    {
                        const int begin = std::max(0, blocked.begin - offset);
                        const int end = std::min(reservations.horizon() - edge.duration, blocked.end - offset);
                        if (begin <= end)
                            edge.blocked.push_back({begin, end});
                    }
                }
                edge.blocked = RouteReservations::merge(std::move(edge.blocked));
                edges.emplace(node.frame, std::move(edge));
            }
            const auto &edge = edges.at(node.frame);
            const auto &next_intervals = reservations.safe_intervals(edge.next);
            for (std::size_t next_index = 0; next_index < next_intervals.size(); ++next_index)
            {
                if ((next_index & 255U) == 0U) reservations.check_budget();
                const auto &destination = next_intervals[next_index];
                int departure = std::max(node.arrival, destination.begin - edge.duration);
                const int end = std::min(interval.end, destination.end - edge.duration);
                if (departure > end)
                    continue;
                auto blocked = std::lower_bound(edge.blocked.begin(), edge.blocked.end(), departure,
                                                 [](const auto &interval, int value) { return interval.end < value; });
                for (; blocked != edge.blocked.end() && blocked->begin <= departure; ++blocked)
                {
                    departure = blocked->end + 1;
                    if (departure > end)
                        break;
                }
                if (departure > end)
                    continue;
                const int arrival = departure + edge.duration;
                const auto key = std::make_pair(edge.next, next_index);
                const auto found = best.find(key);
                if (found != best.end() && found->second <= arrival)
                    continue;
                best[key] = arrival;
                nodes.push_back({edge.next, next_index, arrival, id, departure});
                const int remaining = static_cast<int>(route.size() - 1U - edge.next);
                open.emplace(arrival + remaining, -arrival, nodes.size() - 1U);
            }
        }
        return false;
    }
}
