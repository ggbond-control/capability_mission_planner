#include <capability_mission_planner/offline_map_planner.hpp>

#include <capability_mission_planner/search/a_star.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace capability_mission_planner::offline
{
    namespace
    {
        struct GridPositionHash
        {
            std::size_t operator()(const GridPosition &position) const noexcept
            {
                std::size_t value = std::hash<std::string>{}(position.map_id);
                value ^= static_cast<std::size_t>(static_cast<unsigned int>(position.x)) +
                         0x9e3779b9U + (value << 6U) + (value >> 2U);
                value ^= static_cast<std::size_t>(static_cast<unsigned int>(position.y)) +
                         0x9e3779b9U + (value << 6U) + (value >> 2U);
                return value;
            }
        };

        struct GridAction
        {
        };

        bool contains_all(const CapabilitySet &available, const CapabilitySet &required)
        {
            return std::all_of(required.begin(), required.end(), [&](const auto &capability)
                               { return available.count(capability) != 0U; });
        }

        int seconds_to_ticks(double seconds, double time_step)
        {
            return std::max(1, static_cast<int>(std::ceil(seconds / time_step - 1e-9)));
        }

        double transition_seconds(const MapTransition &transition, const TraversalOptions &options)
        {
            const auto configured = options.transition_seconds.find(transition.type);
            return (configured == options.transition_seconds.end()
                        ? options.default_transition_seconds
                        : configured->second) +
                   options.map_switch_seconds;
        }

        std::string position_key(const GridPosition &position)
        {
            return position.map_id + ':' + std::to_string(position.x) + ':' +
                   std::to_string(position.y);
        }

        std::string cache_key(
            const GridPosition &start,
            const GridPosition &goal,
            const CapabilitySet &capabilities,
            double required_clearance_m,
            double nominal_speed_mps = 0.0)
        {
            std::ostringstream output;
            output << position_key(start) << '>' << position_key(goal) << '|';
            for (const auto &capability : capabilities)
                output << capability << ',';
            output << "|clearance=" << required_clearance_m;
            output << "|speed=" << nominal_speed_mps;
            return output.str();
        }

        std::string estimate_cache_key(
            const GridPosition &start,
            const GridPosition &goal,
            double required_clearance_m,
            double nominal_speed_mps)
        {
            std::ostringstream output;
            output << position_key(start) << '>' << position_key(goal)
                   << "|clearance=" << required_clearance_m
                   << "|speed=" << nominal_speed_mps;
            return output.str();
        }

        class GridEnvironment
        {
        public:
            GridEnvironment(const MapLayer &map, GridPosition goal, int move_ticks,
                            int diagonal_ticks, double required_clearance_m, double obstacle_cost_weight,
                            bool allow_diagonal)
                : _map(map), _goal(std::move(goal)), _move_ticks(move_ticks),
                  _diagonal_ticks(diagonal_ticks), _required_clearance_m(required_clearance_m),
                  _obstacle_cost_weight(obstacle_cost_weight), _allow_diagonal(allow_diagonal) {}

            int admissibleHeuristic(const GridPosition &state) const
            {
                const int diagonal = std::min(std::abs(state.x - _goal.x), std::abs(state.y - _goal.y));
                const int straight = std::abs(state.x - _goal.x) + std::abs(state.y - _goal.y) - 2 * diagonal;
                return diagonal * _diagonal_ticks + straight * _move_ticks;
            }
            bool isSolution(const GridPosition &state) const { return state == _goal; }
            void getNeighbors(
                const GridPosition &state,
                std::vector<search::Neighbor<GridPosition, GridAction, int>> &neighbors) const
            {
                static constexpr std::array<std::array<int, 2>, 8> offsets{{{{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}}, {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}}};
                for (const auto &offset : offsets)
                {
                    const bool diagonal = offset[0] != 0 && offset[1] != 0;
                    if (diagonal && !_allow_diagonal)
                        continue;
                    GridPosition next{state.map_id, state.x + offset[0], state.y + offset[1]};
                    if (!_map.is_traversable(next.x, next.y) ||
                        _map.clearance(next.x, next.y) < _required_clearance_m)
                        continue;
                    if (diagonal && (!_map.is_traversable(state.x + offset[0], state.y) ||
                                     !_map.is_traversable(state.x, state.y + offset[1])))
                        continue;
                    const int move_ticks = diagonal ? _diagonal_ticks : _move_ticks;
                    const int penalty = static_cast<int>(std::lround(
                        _obstacle_cost_weight * move_ticks * _map.cost(next.x, next.y) / 252.0));
                    neighbors.emplace_back(next, GridAction{}, std::max(1, move_ticks + penalty));
                }
            }
            void onExpandNode(const GridPosition &, int, int) const {}
            void onDiscover(const GridPosition &, int, int) const {}

        private:
            const MapLayer &_map;
            GridPosition _goal;
            int _move_ticks;
            int _diagonal_ticks;
            double _required_clearance_m;
            double _obstacle_cost_weight;
            bool _allow_diagonal;
        };

        struct Previous
        {
            std::size_t node = 0;
            bool transition = false;
            std::string transition_id;
            int transition_ticks = 0;
        };

        struct CompactSearchResult
        {
            int cost = 0;
            std::size_t expanded_nodes = 0;
            std::vector<std::pair<int, int>> states;
        };

        CompactSearchResult compact_grid_search(
            const MapLayer &map,
            const GridPosition &start,
            const GridPosition &goal,
            int move_ticks,
            int diagonal_ticks,
            double required_clearance_m,
            double obstacle_cost_weight,
            bool allow_diagonal,
            bool reconstruct)
        {
            const int width = map.width;
            const int height = map.height;
            const auto index = [width](int x, int y)
            {
                return y * width + x;
            };
            const int start_index = index(start.x, start.y);
            const int goal_index = index(goal.x, goal.y);
            const int cell_count = width * height;
            const int infinity = std::numeric_limits<int>::max();
            struct Node
            {
                int index;
                int g;
                int f;
                bool operator<(const Node &other) const
                {
                    if (f != other.f)
                        return f > other.f;
                    return g < other.g;
                }
            };
            std::vector<int> g_score(static_cast<std::size_t>(cell_count), infinity);
            std::vector<int> parent(static_cast<std::size_t>(cell_count), -1);
            std::vector<unsigned char> closed(static_cast<std::size_t>(cell_count), 0U);
            std::priority_queue<Node> open;
            const auto heuristic = [&](int cell)
            {
                const int x = cell % width;
                const int y = cell / width;
                const int diagonal = std::min(std::abs(x - goal.x), std::abs(y - goal.y));
                const int straight = std::abs(x - goal.x) + std::abs(y - goal.y) - 2 * diagonal;
                return diagonal * diagonal_ticks + straight * move_ticks;
            };
            g_score[static_cast<std::size_t>(start_index)] = 0;
            open.push({start_index, 0, heuristic(start_index)});
            static constexpr std::array<std::array<int, 2>, 8> offsets{{{{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}}, {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}}};
            CompactSearchResult result;
            while (!open.empty())
            {
                const auto current = open.top();
                open.pop();
                if (closed[static_cast<std::size_t>(current.index)] != 0U)
                    continue;
                closed[static_cast<std::size_t>(current.index)] = 1U;
                ++result.expanded_nodes;
                if (current.index == goal_index)
                {
                    result.cost = current.g;
                    if (reconstruct)
                    {
                        std::vector<int> chain;
                        for (int cursor = goal_index; cursor >= 0; cursor = parent[static_cast<std::size_t>(cursor)])
                        {
                            chain.push_back(cursor);
                            if (cursor == start_index)
                                break;
                        }
                        if (chain.back() != start_index)
                            throw std::runtime_error("broken grid path predecessor chain");
                        std::reverse(chain.begin(), chain.end());
                        result.states.reserve(chain.size());
                        for (const auto cell : chain)
                            result.states.emplace_back(cell % width, cell / width);
                    }
                    return result;
                }
                const int x = current.index % width;
                const int y = current.index / width;
                for (const auto &offset : offsets)
                {
                    const bool diagonal = offset[0] != 0 && offset[1] != 0;
                    if (diagonal && !allow_diagonal)
                        continue;
                    const int nx = x + offset[0];
                    const int ny = y + offset[1];
                    if (!map.is_traversable(nx, ny) || map.clearance(nx, ny) < required_clearance_m)
                        continue;
                    if (diagonal && (!map.is_traversable(x + offset[0], y) ||
                                     !map.is_traversable(x, y + offset[1])))
                        continue;
                    const int next = index(nx, ny);
                    if (closed[static_cast<std::size_t>(next)] != 0U)
                        continue;
                    const int base = diagonal ? diagonal_ticks : move_ticks;
                    const int penalty = static_cast<int>(std::lround(
                        obstacle_cost_weight * base * map.cost(nx, ny) / 252.0));
                    const int candidate = current.g + std::max(1, base + penalty);
                    if (candidate >= g_score[static_cast<std::size_t>(next)])
                        continue;
                    g_score[static_cast<std::size_t>(next)] = candidate;
                    parent[static_cast<std::size_t>(next)] = current.index;
                    open.push({next, candidate, candidate + heuristic(next)});
                }
            }
            throw std::runtime_error("no path within " + start.map_id);
        }

        std::vector<int> compact_distance_field(
            const MapLayer &map,
            const GridPosition &start,
            const std::vector<int> &target_indices,
            int move_ticks,
            int diagonal_ticks,
            double required_clearance_m,
            double obstacle_cost_weight,
            bool allow_diagonal,
            std::size_t &expanded_nodes)
        {
            const int width = map.width;
            const int height = map.height;
            const int cell_count = width * height;
            const auto index = [width](int x, int y)
            { return y * width + x; };
            const int infinity = std::numeric_limits<int>::max();
            struct Node
            {
                int index;
                int cost;
                bool operator<(const Node &other) const { return cost > other.cost; }
            };
            std::vector<int> distances(static_cast<std::size_t>(cell_count), infinity);
            std::vector<unsigned char> closed(static_cast<std::size_t>(cell_count), 0U);
            std::vector<unsigned char> target(static_cast<std::size_t>(cell_count), 0U);
            std::size_t remaining = 0;
            for (const auto cell : target_indices)
            {
                if (cell < 0 || cell >= cell_count)
                    continue;
                if (target[static_cast<std::size_t>(cell)] == 0U)
                {
                    target[static_cast<std::size_t>(cell)] = 1U;
                    ++remaining;
                }
            }
            std::priority_queue<Node> open;
            const int start_index = index(start.x, start.y);
            distances[static_cast<std::size_t>(start_index)] = 0;
            open.push({start_index, 0});
            static constexpr std::array<std::array<int, 2>, 8> offsets{{{{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}}, {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}}};
            while (!open.empty() && remaining > 0U)
            {
                const auto current = open.top();
                open.pop();
                if (closed[static_cast<std::size_t>(current.index)] != 0U)
                    continue;
                closed[static_cast<std::size_t>(current.index)] = 1U;
                ++expanded_nodes;
                if (target[static_cast<std::size_t>(current.index)] != 0U)
                {
                    target[static_cast<std::size_t>(current.index)] = 0U;
                    --remaining;
                }
                const int x = current.index % width;
                const int y = current.index / width;
                for (const auto &offset : offsets)
                {
                    const bool diagonal = offset[0] != 0 && offset[1] != 0;
                    if (diagonal && !allow_diagonal)
                        continue;
                    const int nx = x + offset[0];
                    const int ny = y + offset[1];
                    if (!map.is_traversable(nx, ny) || map.clearance(nx, ny) < required_clearance_m)
                        continue;
                    if (diagonal && (!map.is_traversable(x + offset[0], y) ||
                                     !map.is_traversable(x, y + offset[1])))
                        continue;
                    const int next = index(nx, ny);
                    if (closed[static_cast<std::size_t>(next)] != 0U)
                        continue;
                    const int base = diagonal ? diagonal_ticks : move_ticks;
                    const int penalty = static_cast<int>(std::lround(
                        obstacle_cost_weight * base * map.cost(nx, ny) / 252.0));
                    const int candidate = current.cost + std::max(1, base + penalty);
                    if (candidate >= distances[static_cast<std::size_t>(next)])
                        continue;
                    distances[static_cast<std::size_t>(next)] = candidate;
                    open.push({next, candidate});
                }
            }
            return distances;
        }
    } // namespace

    namespace
    {
        std::shared_ptr<const MultiMapBundle> make_coarse_bundle(
            const std::shared_ptr<const MultiMapBundle> &source, unsigned int factor)
        {
            if (factor <= 1U)
                return source;
            auto result = std::make_shared<MultiMapBundle>();
            result->directory = source->directory;
            for (const auto &[id, original] : source->maps)
            {
                MapLayer map = original;
                map.width = (original.width + static_cast<int>(factor) - 1) /
                            static_cast<int>(factor);
                map.height = (original.height + static_cast<int>(factor) - 1) /
                             static_cast<int>(factor);
                map.resolution = original.resolution * factor;
                map.traversable.assign(static_cast<std::size_t>(map.width * map.height), 0U);
                map.clearance_m.assign(map.traversable.size(), 0.0F);
                map.inflated_cost.assign(map.traversable.size(), 254U);
                for (int cy = 0; cy < map.height; ++cy)
                {
                    for (int cx = 0; cx < map.width; ++cx)
                    {
                        bool free = true;
                        float clearance = std::numeric_limits<float>::infinity();
                        unsigned char cost = 0U;
                        for (unsigned int dy = 0; dy < factor; ++dy)
                        {
                            for (unsigned int dx = 0; dx < factor; ++dx)
                            {
                                const int x = cx * static_cast<int>(factor) + static_cast<int>(dx);
                                const int y = cy * static_cast<int>(factor) + static_cast<int>(dy);
                                if (x >= original.width || y >= original.height)
                                    continue;
                                free = free && original.is_traversable(x, y);
                                clearance = std::min(clearance, static_cast<float>(original.clearance(x, y)));
                                cost = std::max(cost, original.cost(x, y));
                            }
                        }
                        const auto index = static_cast<std::size_t>(cy * map.width + cx);
                        map.traversable[index] = free ? 1U : 0U;
                        map.clearance_m[index] = std::isfinite(clearance) ? clearance : 0.0F;
                        map.inflated_cost[index] = free ? cost : 254U;
                    }
                }
                result->maps.emplace(id, std::move(map));
            }
            return result;
        }
    } // namespace

    MultiMapPathPlanner::MultiMapPathPlanner(
        std::shared_ptr<const MultiMapBundle> bundle,
        TraversalOptions options)
        : _bundle(std::move(bundle)), _options(std::move(options))
    {
        if (!_bundle)
            throw std::invalid_argument("multi-map bundle must not be null");
        if (!(_options.time_step_seconds > 0.0) || !(_options.nominal_speed_mps > 0.0))
            throw std::invalid_argument("time step and nominal speed must be positive");
    }

    MultiMapPath MultiMapPathPlanner::plan(
        const GridPosition &start,
        const GridPosition &goal,
        const CapabilitySet &capabilities,
        double required_clearance_m,
        double nominal_speed_mps) const
    {
        ++_stats.plan_requests;
        return plan_impl(start, goal, capabilities, required_clearance_m,
                         nominal_speed_mps, false);
    }

    MultiMapPath MultiMapPathPlanner::plan_exact(
        const GridPosition &start,
        const GridPosition &goal,
        const CapabilitySet &capabilities,
        double required_clearance_m,
        double nominal_speed_mps) const
    {
        ++_stats.plan_requests;
        return plan_impl(start, goal, capabilities, required_clearance_m,
                         nominal_speed_mps, true);
    }

    MultiMapPath MultiMapPathPlanner::plan(
        const GridPosition &start,
        const GridPosition &goal,
        const CapabilitySet &capabilities) const
    {
        ++_stats.plan_requests;
        return plan_impl(start, goal, capabilities, 0.0, 0.0, false);
    }

    MultiMapPath MultiMapPathPlanner::plan(
        const GridPosition &start,
        const GridPosition &goal,
        const CapabilitySet &capabilities,
        double required_clearance_m) const
    {
        return plan_impl(start, goal, capabilities, required_clearance_m, 0.0, false);
    }

    MultiMapPath MultiMapPathPlanner::plan_impl(
        const GridPosition &start,
        const GridPosition &goal,
        const CapabilitySet &capabilities,
        double required_clearance_m,
        double nominal_speed_mps,
        bool force_full_resolution) const
    {
        if (required_clearance_m < 0.0)
            throw std::invalid_argument("required clearance must be non-negative");
        const double speed = nominal_speed_mps > 0.0 ? nominal_speed_mps : _options.nominal_speed_mps;
        if (!(speed > 0.0))
            throw std::invalid_argument("nominal speed must be positive");
        if (!_bundle->traversable(start) || !_bundle->traversable(goal))
            throw std::invalid_argument("path endpoints must be traversable");
        const auto full_key = cache_key(start, goal, capabilities, required_clearance_m, speed) +
                              (force_full_resolution ? "|resolution=exact" : "|resolution=coarse");
        const auto full_cached = _cache.find(full_key);
        if (full_cached != _cache.end())
        {
            ++_stats.cache_hits;
            return full_cached->second;
        }

        const auto same_map_path = [&](const GridPosition &from, const GridPosition &to)
        {
            const std::string key = position_key(from) + '>' + position_key(to) +
                                    "|clearance=" + std::to_string(required_clearance_m) +
                                    "|speed=" + std::to_string(speed) +
                                    (force_full_resolution ? "|resolution=exact" : "|resolution=coarse");
            const auto cached = _segment_cache.find(key);
            if (cached != _segment_cache.end())
            {
                ++_stats.cache_hits;
                return cached->second;
            }
            const auto &map = _bundle->map(from.map_id);
            const int move_ticks = seconds_to_ticks(
                map.resolution / speed, _options.time_step_seconds);
            const int diagonal_ticks = seconds_to_ticks(
                std::sqrt(2.0) * map.resolution / speed,
                _options.time_step_seconds);
            if (map.clearance(from.x, from.y) < required_clearance_m ||
                map.clearance(to.x, to.y) < required_clearance_m)
                throw std::runtime_error("path endpoint has insufficient clearance");
            const auto search_start = std::chrono::steady_clock::now();
            const auto result = compact_grid_search(map, from, to, move_ticks,
                                                    diagonal_ticks, required_clearance_m, _options.obstacle_cost_weight,
                                                    _options.allow_diagonal, true);
            const auto search_end = std::chrono::steady_clock::now();
            ++_stats.grid_searches;
            ++_stats.a_star_searches;
            _stats.expanded_nodes += result.expanded_nodes;
            _stats.grid_search_seconds += std::chrono::duration<double>(
                                              search_end - search_start)
                                              .count();
            MultiMapPath path;
            path.travel_ticks = result.cost;
            path.steps.reserve(result.states.size());
            int arrival = 0;
            for (std::size_t i = 0; i < result.states.size(); ++i)
            {
                if (i > 0U)
                {
                    const auto &previous = result.states[i - 1U];
                    const auto &current = result.states[i];
                    const bool diagonal = previous.first != current.first &&
                                          previous.second != current.second;
                    const int base = diagonal ? diagonal_ticks : move_ticks;
                    const int penalty = static_cast<int>(std::lround(
                        _options.obstacle_cost_weight * base * map.cost(current.first, current.second) / 252.0));
                    arrival += std::max(1, base + penalty);
                }
                path.steps.push_back({{from.map_id, result.states[i].first,
                                       result.states[i].second},
                                      arrival,
                                      {}});
            }
            // Final full-resolution paths are retained by the mission route itself;
            // caching another copy would substantially increase RSS on large jobs.
            if (!force_full_resolution)
                _segment_cache.emplace(key, path);
            return path;
        };

        if (start.map_id == goal.map_id)
        {
            if (!force_full_resolution && _options.downsample_costmap &&
                _options.coarse_search_factor > 1U)
            {
                if (!_coarse_bundle)
                    _coarse_bundle = make_coarse_bundle(_bundle, _options.coarse_search_factor);
                if (!_coarse_planner)
                {
                    TraversalOptions coarse_options = _options;
                    coarse_options.downsample_costmap = false;
                    coarse_options.coarse_search_factor = 1U;
                    _coarse_planner = std::make_shared<MultiMapPathPlanner>(
                        _coarse_bundle, coarse_options);
                }
                const auto coarse_start = GridPosition{start.map_id,
                                                       start.x / static_cast<int>(_options.coarse_search_factor),
                                                       start.y / static_cast<int>(_options.coarse_search_factor)};
                const auto coarse_goal = GridPosition{goal.map_id,
                                                      goal.x / static_cast<int>(_options.coarse_search_factor),
                                                      goal.y / static_cast<int>(_options.coarse_search_factor)};
                const auto coarse_path = _coarse_planner->plan(
                    coarse_start, coarse_goal, capabilities, required_clearance_m, speed);
                MultiMapPath output;
                output.travel_ticks = coarse_path.travel_ticks;
                const auto expand = [&](const GridPosition &position)
                {
                    const int factor = static_cast<int>(_options.coarse_search_factor);
                    const int x = std::min(position.x * factor + factor / 2, _bundle->map(position.map_id).width - 1);
                    const int y = std::min(position.y * factor + factor / 2, _bundle->map(position.map_id).height - 1);
                    return GridPosition{position.map_id, x, y};
                };
                for (std::size_t i = 0; i < coarse_path.steps.size(); ++i)
                {
                    const auto &step = coarse_path.steps[i];
                    // Preserve exact segment endpoints: CBS and route concatenation rely
                    // on the next segment starting at the previous task position.
                    const auto position = i == 0U ? start : (i + 1U == coarse_path.steps.size() ? goal : expand(step.position));
                    output.steps.push_back({position, step.arrival_tick, step.transition_id});
                }
                _cache.emplace(full_key, output);
                return output;
            }
            // same_map_path owns the reusable segment cache. Avoid storing a second
            // full-path copy for same-map queries, which is significant for dense
            // scenarios with many final route segments.
            return same_map_path(start, goal);
        }

        std::vector<GridPosition> nodes{start, goal};
        const auto add_node = [&](const GridPosition &position)
        {
            if (std::find(nodes.begin(), nodes.end(), position) == nodes.end())
                nodes.push_back(position);
        };
        for (const auto &transition : _bundle->transitions)
        {
            const auto requirement = _options.transition_requirements.find(transition.type);
            if (requirement != _options.transition_requirements.end() &&
                !contains_all(capabilities, requirement->second))
            {
                continue;
            }
            if (_bundle->map(transition.from_cell.map_id).clearance(transition.from_cell.x, transition.from_cell.y) < required_clearance_m ||
                _bundle->map(transition.to_cell.map_id).clearance(transition.to_cell.x, transition.to_cell.y) < required_clearance_m)
                continue;
            add_node(transition.from_cell);
            add_node(transition.to_cell);
        }

        const auto node_index = [&](const GridPosition &position)
        {
            const auto found = std::find(nodes.begin(), nodes.end(), position);
            return found == nodes.end() ? nodes.size()
                                        : static_cast<std::size_t>(std::distance(nodes.begin(), found));
        };
        std::vector<int> distance(nodes.size(), std::numeric_limits<int>::max());
        std::vector<Previous> previous(nodes.size());
        std::vector<bool> has_previous(nodes.size(), false);
        std::vector<bool> visited(nodes.size(), false);
        distance[0] = 0;

        for (std::size_t iteration = 0; iteration < nodes.size(); ++iteration)
        {
            std::size_t current = nodes.size();
            for (std::size_t i = 0; i < nodes.size(); ++i)
            {
                if (!visited[i] && distance[i] != std::numeric_limits<int>::max() &&
                    (current == nodes.size() || distance[i] < distance[current]))
                {
                    current = i;
                }
            }
            if (current == nodes.size() || current == 1U)
                break;
            visited[current] = true;

            for (std::size_t next = 0; next < nodes.size(); ++next)
            {
                if (next == current || nodes[next].map_id != nodes[current].map_id)
                    continue;
                try
                {
                    const auto segment = same_map_path(nodes[current], nodes[next]);
                    const int candidate = distance[current] + segment.travel_ticks;
                    if (candidate < distance[next])
                    {
                        distance[next] = candidate;
                        previous[next] = Previous{current, false, {}, 0};
                        has_previous[next] = true;
                    }
                }
                catch (const std::runtime_error &)
                {
                }
            }
            for (const auto &transition : _bundle->transitions)
            {
                if (transition.from_cell != nodes[current])
                    continue;
                const auto requirement = _options.transition_requirements.find(transition.type);
                if (requirement != _options.transition_requirements.end() &&
                    !contains_all(capabilities, requirement->second))
                {
                    continue;
                }
                const auto next = node_index(transition.to_cell);
                if (next == nodes.size())
                    continue;
                const int transition_ticks = seconds_to_ticks(
                    transition_seconds(transition, _options), _options.time_step_seconds);
                const int candidate = distance[current] + transition_ticks;
                if (candidate < distance[next])
                {
                    distance[next] = candidate;
                    previous[next] = Previous{current, true, transition.id, transition_ticks};
                    has_previous[next] = true;
                }
            }
        }

        if (distance[1] == std::numeric_limits<int>::max())
            throw std::runtime_error("no path from " + start.map_id + " to " + goal.map_id);

        std::vector<std::size_t> chain{1U};
        while (chain.back() != 0U)
        {
            if (!has_previous[chain.back()])
                throw std::logic_error("broken multi-map path predecessor chain");
            chain.push_back(previous[chain.back()].node);
        }
        std::reverse(chain.begin(), chain.end());

        MultiMapPath output;
        output.steps.push_back({start, 0, {}});
        int elapsed = 0;
        for (std::size_t i = 1; i < chain.size(); ++i)
        {
            const auto to = chain[i];
            const auto from = chain[i - 1U];
            const auto &edge = previous[to];
            if (edge.transition)
            {
                elapsed += edge.transition_ticks;
                output.steps.push_back({nodes[to], elapsed, edge.transition_id});
            }
            else
            {
                const auto segment = same_map_path(nodes[from], nodes[to]);
                for (std::size_t step = 1; step < segment.steps.size(); ++step)
                {
                    output.steps.push_back({segment.steps[step].position,
                                            elapsed + segment.steps[step].arrival_tick,
                                            {}});
                }
                elapsed += segment.travel_ticks;
            }
        }
        output.travel_ticks = elapsed;
        _cache.emplace(full_key, output);
        return output;
    }

    void MultiMapPathPlanner::precompute_estimates(
        const std::vector<GridPosition> &positions,
        const std::vector<PathQueryProfile> &profiles) const
    {
        if (positions.empty() || profiles.empty())
            return;
        const auto source_bundle = [&]()
        {
            if (_options.downsample_costmap && _options.coarse_search_factor > 1U)
            {
                if (!_coarse_bundle)
                    _coarse_bundle = make_coarse_bundle(_bundle, _options.coarse_search_factor);
                return _coarse_bundle;
            }
            return _bundle;
        }();
        const int factor = _options.downsample_costmap
                               ? static_cast<int>(_options.coarse_search_factor)
                               : 1;
        std::vector<std::string> map_ids;
        for (const auto &[id, map] : _bundle->maps)
        {
            (void)map;
            map_ids.push_back(id);
        }
        for (const auto &profile : profiles)
        {
            const double speed = profile.nominal_speed_mps > 0.0
                                     ? profile.nominal_speed_mps
                                     : _options.nominal_speed_mps;
            if (!(speed > 0.0) || profile.required_clearance_m < 0.0)
                continue;
            for (const auto &map_id : map_ids)
            {
                const auto &original_map = _bundle->map(map_id);
                const auto &map = source_bundle->map(map_id);
                std::vector<GridPosition> endpoints;
                for (const auto &position : positions)
                    if (position.map_id == map_id &&
                        std::find(endpoints.begin(), endpoints.end(), position) == endpoints.end())
                        endpoints.push_back(position);
                if (endpoints.empty())
                    continue;
                const int move_ticks = seconds_to_ticks(
                    map.resolution / speed, _options.time_step_seconds);
                const int diagonal_ticks = seconds_to_ticks(
                    std::sqrt(2.0) * map.resolution / speed, _options.time_step_seconds);
                std::vector<int> target_indices;
                target_indices.reserve(endpoints.size());
                std::vector<GridPosition> projected;
                projected.reserve(endpoints.size());
                for (const auto &endpoint : endpoints)
                {
                    if (original_map.clearance(endpoint.x, endpoint.y) < profile.required_clearance_m)
                        continue;
                    const GridPosition coarse{map_id, endpoint.x / factor, endpoint.y / factor};
                    if (!map.is_traversable(coarse.x, coarse.y) ||
                        map.clearance(coarse.x, coarse.y) < profile.required_clearance_m)
                        continue;
                    projected.push_back(endpoint);
                    target_indices.push_back(coarse.y * map.width + coarse.x);
                }
                for (const auto &source : projected)
                {
                    bool complete = true;
                    for (const auto &target_position : projected)
                    {
                        if (_estimate_cache.find(estimate_cache_key(source, target_position,
                                                                    profile.required_clearance_m, speed)) == _estimate_cache.end())
                        {
                            complete = false;
                            break;
                        }
                    }
                    if (complete)
                        continue;
                    const GridPosition coarse_source{map_id, source.x / factor, source.y / factor};
                    const auto search_start = std::chrono::steady_clock::now();
                    std::size_t expanded = 0;
                    const auto field = compact_distance_field(map, coarse_source, target_indices,
                                                              move_ticks, diagonal_ticks, profile.required_clearance_m,
                                                              _options.obstacle_cost_weight, _options.allow_diagonal, expanded);
                    const auto search_end = std::chrono::steady_clock::now();
                    ++_stats.grid_searches;
                    ++_stats.distance_field_searches;
                    _stats.distance_field_seconds += std::chrono::duration<double>(
                                                         search_end - search_start)
                                                         .count();
                    _stats.expanded_nodes += expanded;
                    for (std::size_t i = 0; i < projected.size(); ++i)
                    {
                        const auto &target_position = projected[i];
                        const GridPosition coarse_target{map_id,
                                                         target_position.x / factor, target_position.y / factor};
                        const int distance = field[static_cast<std::size_t>(
                            coarse_target.y * map.width + coarse_target.x)];
                        if (distance == std::numeric_limits<int>::max())
                            continue;
                        _estimate_cache.emplace(estimate_cache_key(source, target_position,
                                                                   profile.required_clearance_m, speed),
                                                distance);
                    }
                }
            }
        }
    }

    int MultiMapPathPlanner::distance(
        const GridPosition &start,
        const GridPosition &goal,
        const CapabilitySet &capabilities) const
    {
        return plan(start, goal, capabilities).travel_ticks;
    }

    int MultiMapPathPlanner::estimate_distance(
        const GridPosition &start,
        const GridPosition &goal,
        const CapabilitySet &capabilities,
        double required_clearance_m,
        double nominal_speed_mps) const
    {
        ++_stats.estimate_requests;
        const double speed = nominal_speed_mps > 0.0 ? nominal_speed_mps : _options.nominal_speed_mps;
        const auto estimate_cached = _estimate_cache.find(estimate_cache_key(
            start, goal, required_clearance_m, speed));
        if (estimate_cached != _estimate_cache.end())
        {
            ++_stats.cache_hits;
            return estimate_cached->second;
        }
        if (!_options.downsample_costmap || _options.coarse_search_factor <= 1U ||
            start.map_id != goal.map_id)
            return plan(start, goal, capabilities, required_clearance_m,
                        nominal_speed_mps)
                .travel_ticks;
        if (!_coarse_bundle)
            _coarse_bundle = make_coarse_bundle(_bundle, _options.coarse_search_factor);
        const auto coarse_position = [&](const GridPosition &position)
        {
            return GridPosition{position.map_id,
                                position.x / static_cast<int>(_options.coarse_search_factor),
                                position.y / static_cast<int>(_options.coarse_search_factor)};
        };
        try
        {
            if (!_coarse_planner)
            {
                auto coarse_options = _options;
                coarse_options.downsample_costmap = false;
                coarse_options.coarse_search_factor = 1U;
                _coarse_planner = std::make_shared<MultiMapPathPlanner>(_coarse_bundle, coarse_options);
            }
            return _coarse_planner->plan(coarse_position(start), coarse_position(goal),
                                         capabilities, required_clearance_m, nominal_speed_mps)
                .travel_ticks;
        }
        catch (const std::runtime_error &)
        {
            return distance(start, goal, capabilities, required_clearance_m);
        }
    }

    PathPlannerStats MultiMapPathPlanner::stats() const
    {
        auto result = _stats;
        if (_coarse_planner)
        {
            const auto coarse = _coarse_planner->stats();
            result.estimate_requests += coarse.estimate_requests;
            result.plan_requests += coarse.plan_requests;
            result.cache_hits += coarse.cache_hits;
            result.grid_searches += coarse.grid_searches;
            result.a_star_searches += coarse.a_star_searches;
            result.expanded_nodes += coarse.expanded_nodes;
            result.grid_search_seconds += coarse.grid_search_seconds;
            result.distance_field_searches += coarse.distance_field_searches;
            result.distance_field_seconds += coarse.distance_field_seconds;
        }
        return result;
    }

    void MultiMapPathPlanner::reset_stats() const
    {
        _stats = {};
        if (_coarse_planner)
            _coarse_planner->reset_stats();
    }

    int MultiMapPathPlanner::distance(
        const GridPosition &start,
        const GridPosition &goal,
        const CapabilitySet &capabilities,
        double required_clearance_m) const
    {
        return plan(start, goal, capabilities, required_clearance_m).travel_ticks;
    }
} // namespace capability_mission_planner::offline
