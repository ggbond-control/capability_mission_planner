#include "multi_map_coordinator.hpp"
#include "route_reservations.hpp"
#include "safe_interval_scheduler.hpp"

#include <capability_mission_planner/search/conflict_based_search.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <unordered_set>

namespace capability_mission_planner::offline
{
    namespace
    {
        struct State
        {
            int tick = 0;
            std::size_t frame = 0;

            bool operator==(const State &other) const
            {
                return tick == other.tick && frame == other.frame;
            }
        };

        std::size_t grid_position_hash(const GridPosition &position)
        {
            std::size_t value = std::hash<std::string>{}(position.map_id);
            value ^= static_cast<std::size_t>(static_cast<unsigned int>(position.x)) * 73856093U;
            value ^= static_cast<std::size_t>(static_cast<unsigned int>(position.y)) * 19349663U;
            return value;
        }

        struct StateHash
        {
            std::size_t operator()(const State &state) const noexcept
            {
                return static_cast<std::size_t>(state.tick) * 83492791U ^
                       state.frame * 2654435761U;
            }
        };

        enum class Action
        {
            Wait,
            Advance
        };

        struct VertexConstraint
        {
            int tick = 0;
            GridPosition position;
            bool operator==(const VertexConstraint &other) const
            {
                return tick == other.tick && position == other.position;
            }
        };
        struct VertexHash
        {
            std::size_t operator()(const VertexConstraint &value) const noexcept
            {
                return grid_position_hash(value.position) ^
                       static_cast<std::size_t>(value.tick) * 83492791U;
            }
        };

        struct ResourceConstraint
        {
            int tick = 0;
            std::string resource;
            bool operator==(const ResourceConstraint &other) const
            {
                return tick == other.tick && resource == other.resource;
            }
        };
        struct ResourceHash
        {
            std::size_t operator()(const ResourceConstraint &value) const noexcept
            {
                return std::hash<std::string>{}(value.resource) ^
                       (static_cast<std::size_t>(value.tick) * 2654435761U);
            }
        };

        struct EdgeConstraint
        {
            int tick = 0;
            GridPosition from;
            GridPosition to;
            bool operator==(const EdgeConstraint &other) const
            {
                return tick == other.tick && from == other.from && to == other.to;
            }
        };
        struct EdgeHash
        {
            std::size_t operator()(const EdgeConstraint &value) const noexcept
            {
                return grid_position_hash(value.from) ^ (grid_position_hash(value.to) << 1U) ^
                       static_cast<std::size_t>(value.tick) * 83492791U;
            }
        };

        struct Constraints
        {
            std::unordered_set<VertexConstraint, VertexHash> vertex;
            std::unordered_set<ResourceConstraint, ResourceHash> resource;
            std::unordered_set<EdgeConstraint, EdgeHash> edge;

            void add(const Constraints &other)
            {
                vertex.insert(other.vertex.begin(), other.vertex.end());
                resource.insert(other.resource.begin(), other.resource.end());
                edge.insert(other.edge.begin(), other.edge.end());
            }
            bool overlap(const Constraints &other) const
            {
                for (const auto &item : other.vertex)
                    if (vertex.count(item) != 0U)
                        return true;
                for (const auto &item : other.resource)
                    if (resource.count(item) != 0U)
                        return true;
                for (const auto &item : other.edge)
                    if (edge.count(item) != 0U)
                        return true;
                return false;
            }
        };

        struct Conflict
        {
            enum class Type
            {
                Vertex,
                Edge,
                Resource
            } type = Type::Vertex;
            int tick = 0;
            std::size_t first = 0;
            std::size_t second = 0;
            GridPosition first_from;
            GridPosition first_to;
            GridPosition second_from;
            GridPosition second_to;
            std::string resource;
        };

        using Plan = search::PlanResult<State, Action, int>;

        const State &state_at(const Plan &plan, std::size_t tick)
        {
            return tick < plan.states.size() ? plan.states[tick].first : plan.states.back().first;
        }

        std::string edge_resource(const GridPosition &first, const GridPosition &second)
        {
            const auto &low = second < first ? second : first;
            const auto &high = second < first ? first : second;
            return "edge:" + low.map_id + ':' + std::to_string(low.x) + ':' +
                   std::to_string(low.y) + ':' + std::to_string(high.x) + ':' +
                   std::to_string(high.y);
        }

        bool is_in_transit(const std::string &resource)
        {
            return resource.compare(0, 5U, "edge:") == 0 ||
                   resource.compare(0, 11U, "transition:") == 0;
        }

        void append_segment(std::vector<RouteFrame> &frames, const MultiMapPath &segment)
        {
            if (segment.steps.empty())
                return;
            if (frames.back().position != segment.steps.front().position)
                throw std::logic_error("route segment does not start at the current position");
            for (std::size_t i = 1; i < segment.steps.size(); ++i)
            {
                const auto &from = segment.steps[i - 1U];
                const auto &to = segment.steps[i];
                const int duration = to.arrival_tick - from.arrival_tick;
                if (duration <= 0)
                    throw std::logic_error("route segment arrival ticks must increase");
                const std::string resource = to.transition_id.empty()
                                                 ? edge_resource(from.position, to.position)
                                                 : "transition:" + to.transition_id;
                for (int tick = 1; tick < duration; ++tick)
                    frames.push_back({from.position, resource});
                frames.push_back({to.position, {}});
            }
        }

        std::vector<RouteFrame> make_frames(const MappedRobot &robot, const MappedRobotRoute &route,
                                            const TraversalOptions &options)
        {
            std::vector<RouteFrame> frames{{robot.start, {}}};
            std::size_t segment = 0;
            for (const auto &stop : route.stops)
            {
                if (segment >= route.segments.size())
                    throw std::logic_error("missing path segment for route stop");
                append_segment(frames, route.segments[segment++]);
                for (int tick = 0; tick < stop.service_ticks; ++tick)
                    frames.push_back({stop.location, {}});
            }
            if (segment < route.segments.size())
                append_segment(frames, route.segments[segment++]);
            if (segment != route.segments.size())
                throw std::logic_error("unexpected extra path segments in route");
            for (std::size_t i = 0; i < frames.size(); ++i)
            {
                if (!frames[i].resource.empty())
                    continue;
                for (const auto &resource : options.shared_resources)
                {
                    if (std::find(resource.cells.begin(), resource.cells.end(), frames[i].position) !=
                        resource.cells.end())
                    {
                        frames[i].resource = "region:" + resource.id;
                        break;
                    }
                }
            }
            for (const auto &resource : options.shared_resources)
            {
                const int buffer = std::max(0, static_cast<int>(std::ceil(
                                                   resource.buffer_seconds / options.time_step_seconds)));
                const std::string id = "region:" + resource.id;
                for (int i = 0; i < static_cast<int>(frames.size()); ++i)
                {
                    if (frames[i].resource != id)
                        continue;
                    for (int offset = 1; offset <= buffer; ++offset)
                    {
                        if (i - offset > 0 && frames[i - offset].resource.empty())
                            frames[i - offset].resource = id;
                        if (i + offset < static_cast<int>(frames.size()) && frames[i + offset].resource.empty())
                            frames[i + offset].resource = id;
                    }
                }
            }
            if (options.resource_buffer_seconds > 0.0)
            {
                const int buffer = static_cast<int>(std::ceil(
                    options.resource_buffer_seconds / options.time_step_seconds));
                for (std::size_t i = 0; i < frames.size(); ++i)
                {
                    if (frames[i].resource.empty())
                        continue;
                    const auto resource = frames[i].resource;
                    for (int offset = 1; offset <= buffer; ++offset)
                    {
                        if (i > static_cast<std::size_t>(offset) && frames[i - offset].resource.empty())
                            frames[i - offset].resource = resource;
                        if (i + static_cast<std::size_t>(offset) < frames.size() && frames[i + offset].resource.empty())
                            frames[i + offset].resource = resource;
                    }
                }
            }
            if (!frames.empty() && !frames.back().resource.empty())
                frames.push_back({frames.back().position, {}});
            return frames;
        }

        class Environment
        {
        public:
            Environment(std::vector<std::vector<RouteFrame>> frames,
                        const MultiMapBundle &bundle, const std::vector<MappedRobot> &robots,
                        CoordinationStats *stats)
                : _frames(std::move(frames)), _robots(robots), _stats(stats)
            {
                for (auto &route : _frames)
                    for (auto &frame : route)
                    {
                        const auto &map = bundle.map(frame.position.map_id);
                        const auto root = map.local_to_root(map.grid_to_local(frame.position));
                        frame.root_x = root.x;
                        frame.root_y = root.y;
                    }
            }

            void set_cbs_phase(bool value) { _cbs_phase = value; }
            void set_stats(CoordinationStats *stats) { _stats = stats; }
            void set_budget(std::function<void()> budget) { _check_budget = std::move(budget); }
            void check_budget() const { if (_check_budget) _check_budget(); }

            void prepare_reservations(std::size_t agent, const std::vector<Plan> &plans,
                                      const std::vector<std::size_t> &fixed, int horizon)
            {
                std::vector<ReservationTrack> tracks;
                for (const auto other : fixed)
                {
                    check_budget();
                    ReservationTrack track{other, {}};
                    track.frames.reserve(plans[other].states.size());
                    for (const auto &state : plans[other].states)
                        track.frames.push_back(state.first.frame);
                    tracks.push_back(std::move(track));
                }
                _reservations = std::make_unique<RouteReservations>(_frames, _robots, agent, tracks, horizon, _stats, _check_budget);
            }

            RouteReservations &reservations() { return *_reservations; }
            const std::vector<RouteFrame> &frames(std::size_t agent) const { return _frames.at(agent); }
            CoordinationStats *stats() { return _stats; }

            const RouteFrame &route_frame(std::size_t agent, const State &state) const
            {
                return _frames.at(agent).at(state.frame);
            }

            void setLowLevelContext(std::size_t agent, const Constraints *constraints)
            {
                _agent = agent;
                _constraints = constraints;
                _last_goal_constraint = -1;
                const auto &goal = _frames[agent].back().position;
                for (const auto &constraint : constraints->vertex)
                    if (constraint.position == goal)
                        _last_goal_constraint = std::max(_last_goal_constraint, constraint.tick);
                if (_stats)
                {
                    if (_stats->prioritized_expanded_nodes_by_robot.size() < _frames.size())
                    {
                        _stats->prioritized_expanded_nodes_by_robot.assign(_frames.size(), 0U);
                        _stats->prioritized_searches_by_robot.assign(_frames.size(), 0U);
                        _stats->route_frames_by_robot.resize(_frames.size());
                        _stats->prioritized_frame_expansions_by_robot.resize(_frames.size());
                        for (std::size_t i = 0; i < _frames.size(); ++i)
                            _stats->route_frames_by_robot[i] = _frames[i].size();
                    }
                    if (_cbs_phase)
                        ++_stats->cbs_low_level_searches;
                    else
                    {
                        ++_stats->prioritized_low_level_searches;
                        ++_stats->prioritized_searches_by_robot[agent];
                    }
                }
            }

            int admissibleHeuristic(const State &state) const
            {
                return static_cast<int>(_frames[_agent].size() - 1U - state.frame);
            }
            bool isSolution(const State &state) const
            {
                return state.frame + 1U == _frames[_agent].size() && state.tick > _last_goal_constraint;
            }

            void getNeighbors(const State &state,
                              std::vector<search::Neighbor<State, Action, int>> &neighbors) const
            {
                // A robot may wait at a cell, but not midway through an edge or transition.
                if (!is_in_transit(route_frame(_agent, state).resource))
                {
                    State waiting = state;
                    ++waiting.tick;
                    if (allowed(state, waiting))
                        neighbors.emplace_back(std::move(waiting), Action::Wait, 1);
                }
                if (state.frame + 1U < _frames[_agent].size())
                {
                    State next{state.tick + 1, state.frame + 1U};
                    if (allowed(state, next))
                        neighbors.emplace_back(std::move(next), Action::Advance, 1);
                }
            }

            bool getFirstConflict(const std::vector<Plan> &plans, Conflict &output) const
            {
                const auto started = std::chrono::steady_clock::now();
                const auto finish = [&]()
                {
                    if (!_stats)
                        return;
                    ++_stats->conflict_checks;
                    _stats->conflict_check_seconds += std::chrono::duration<double>(
                                                          std::chrono::steady_clock::now() - started)
                                                          .count();
                };
                std::size_t horizon = 0;
                for (const auto &plan : plans)
                    horizon = std::max(horizon, plan.states.size());
                for (std::size_t tick = 0; tick < horizon; ++tick)
                {
                    if ((tick & 255U) == 0U) check_budget();
                    for (std::size_t a = 0; a < plans.size(); ++a)
                    {
                        for (std::size_t b = a + 1U; b < plans.size(); ++b)
                        {
                            const auto &first = state_at(plans[a], tick);
                            const auto &second = state_at(plans[b], tick);
                            const auto &first_frame = route_frame(a, first);
                            const auto &second_frame = route_frame(b, second);
                            if (!first_frame.resource.empty() && first_frame.resource == second_frame.resource)
                            {
                                output = Conflict{Conflict::Type::Resource, static_cast<int>(tick), a, b, {}, {}, {}, {}, first_frame.resource};
                                finish();
                                return true;
                            }
                            if (positions_too_close(first_frame, second_frame, a, b))
                            {
                                output = Conflict{Conflict::Type::Vertex, static_cast<int>(tick), a, b, first_frame.position, first_frame.position, second_frame.position, second_frame.position, {}};
                                finish();
                                return true;
                            }
                            if (tick + 1U >= horizon)
                                continue;
                            const auto &first_next = state_at(plans[a], tick + 1U);
                            const auto &second_next = state_at(plans[b], tick + 1U);
                            const auto &first_next_frame = route_frame(a, first_next);
                            const auto &second_next_frame = route_frame(b, second_next);
                            if (swept_paths_too_close(first_frame, first_next_frame,
                                                      second_frame, second_next_frame, a, b))
                            {
                                output = Conflict{Conflict::Type::Edge, static_cast<int>(tick), a, b, first_frame.position, first_next_frame.position, second_frame.position, second_next_frame.position, {}};
                                finish();
                                return true;
                            }
                        }
                    }
                }
                finish();
                return false;
            }

            void createConstraintsFromConflict(const Conflict &conflict,
                                               std::map<std::size_t, Constraints> &output) const
            {
                if (conflict.type == Conflict::Type::Vertex)
                {
                    Constraints first;
                    first.vertex.insert({conflict.tick, conflict.first_to});
                    output[conflict.first] = first;
                    Constraints second;
                    second.vertex.insert({conflict.tick, conflict.second_to});
                    output[conflict.second] = second;
                }
                else if (conflict.type == Conflict::Type::Resource)
                {
                    Constraints constraint;
                    constraint.resource.insert({conflict.tick, conflict.resource});
                    output[conflict.first] = constraint;
                    output[conflict.second] = constraint;
                }
                else
                {
                    Constraints first;
                    first.edge.insert({conflict.tick, conflict.first_from, conflict.first_to});
                    output[conflict.first] = first;
                    Constraints second;
                    second.edge.insert({conflict.tick, conflict.second_from, conflict.second_to});
                    output[conflict.second] = second;
                }
            }

            void onExpandHighLevelNode(int)
            {
                if (_stats && _cbs_phase)
                    ++_stats->cbs_high_level_expanded_nodes;
            }
            void onExpandLowLevelNode(const State &state, int, int)
            {
                if (!_stats)
                    return;
                if (_cbs_phase)
                    ++_stats->cbs_low_level_expanded_nodes;
                else
                {
                    ++_stats->prioritized_low_level_expanded_nodes;
                    ++_stats->prioritized_expanded_nodes_by_robot[_agent];
                    auto &frames = _stats->prioritized_frame_expansions_by_robot[_agent];
                    if (frames.size() <= state.frame)
                        frames.resize(state.frame + 1U, 0U);
                    ++frames[state.frame];
                }
            }

        private:
            double safety_distance(std::size_t first, std::size_t second) const
            {
                return _robots[first].footprint_radius_m + _robots[second].footprint_radius_m +
                       _robots[first].safety_margin_m + _robots[second].safety_margin_m + 1e-9;
            }

            bool positions_too_close(const RouteFrame &first, const RouteFrame &second,
                                     std::size_t a, std::size_t b) const
            {
                if (first.position.map_id != second.position.map_id)
                    return false;
                const double dx = first.root_x - second.root_x;
                const double dy = first.root_y - second.root_y;
                const double radius = safety_distance(a, b);
                return dx * dx + dy * dy <= radius * radius;
            }

            bool swept_paths_too_close(const RouteFrame &af, const RouteFrame &at,
                                       const RouteFrame &bf, const RouteFrame &bt,
                                       std::size_t a, std::size_t b) const
            {
                if (af.position.map_id != at.position.map_id ||
                    bf.position.map_id != bt.position.map_id || af.position.map_id != bf.position.map_id)
                    return false;
                const double dx = af.root_x - bf.root_x, dy = af.root_y - bf.root_y;
                const double vx = (at.root_x - af.root_x) - (bt.root_x - bf.root_x);
                const double vy = (at.root_y - af.root_y) - (bt.root_y - bf.root_y);
                const double vv = vx * vx + vy * vy;
                const double t = vv > 1e-12 ? std::clamp(-(dx * vx + dy * vy) / vv, 0.0, 1.0) : 0.0;
                const double x = dx + t * vx, y = dy + t * vy;
                const double radius = safety_distance(a, b);
                return x * x + y * y <= radius * radius;
            }

            bool allowed(const State &from, const State &to) const
            {
                const auto &from_frame = route_frame(_agent, from);
                const auto &to_frame = route_frame(_agent, to);
                if (_constraints->vertex.count({to.tick, to_frame.position}) != 0U)
                    return false;
                if (!to_frame.resource.empty() &&
                    _constraints->resource.count({to.tick, to_frame.resource}) != 0U)
                    return false;
                if (_constraints->edge.count({from.tick, from_frame.position, to_frame.position}) != 0U)
                    return false;
                return true;
            }

            std::vector<std::vector<RouteFrame>> _frames;
            const std::vector<MappedRobot> &_robots;
            CoordinationStats *_stats = nullptr;
            std::function<void()> _check_budget;
            std::unique_ptr<RouteReservations> _reservations;
            bool _cbs_phase = false;
            std::size_t _agent = 0;
            const Constraints *_constraints = nullptr;
            int _last_goal_constraint = -1;
        };

        bool prioritized_schedule(Environment &environment, const std::vector<State> &starts,
                                  int maximum_tick, std::vector<Plan> &plans)
        {
            plans.assign(starts.size(), {});
            std::vector<std::size_t> fixed;
            for (std::size_t agent = 0; agent < starts.size(); ++agent)
            {
                environment.prepare_reservations(agent, plans, fixed, maximum_tick);
                Constraints constraints;
                environment.setLowLevelContext(agent, &constraints);
                std::vector<std::size_t> route_frames;
                if (!schedule_safe_intervals(environment.reservations(), environment.frames(agent), agent,
                                             route_frames, environment.stats()))
                    return false;
                auto &plan = plans[agent];
                for (std::size_t tick = 0; tick < route_frames.size(); ++tick)
                    plan.states.push_back({{static_cast<int>(tick), route_frames[tick]}, static_cast<int>(tick)});
                plan.cost = static_cast<int>(route_frames.size() - 1U);
                plan.fmin = plan.cost;
                fixed.push_back(agent);
            }
            Conflict conflict;
            return !environment.getFirstConflict(plans, conflict);
        }

        struct RepairBudgetReached {};

        // One transaction: change at most three schedules, keep all others frozen.
        // Failed, slower, or over-budget trials never replace the baseline.
        void repair_critical_schedule(Environment &environment, const TraversalOptions &options,
                                      int horizon, std::vector<Plan> &plans, CoordinationStats &stats)
        {
            if (!options.coordination_local_repair || plans.size() < 2)
                return;
            const auto critical = static_cast<std::size_t>(std::distance(plans.begin(),
                std::max_element(plans.begin(), plans.end(), [](const auto &a, const auto &b)
                                 { return a.cost < b.cost; })));
            const auto mandatory = environment.frames(critical).size() - 1U;
            const auto wait = static_cast<std::size_t>(plans[critical].cost) - mandatory;
            if (wait < std::max<std::size_t>(2, mandatory / 20U))
                return;
            std::vector<std::size_t> blockers;
            const auto &counts = stats.blocking_departure_ticks_by_robot.at(critical);
            for (std::size_t r = 0; r < plans.size(); ++r)
                if (r != critical && counts[r]) blockers.push_back(r);
            std::stable_sort(blockers.begin(), blockers.end(), [&](auto a, auto b)
                             { return counts[a] > counts[b]; });
            if (blockers.empty()) return;
            blockers.resize(std::min<std::size_t>(2, blockers.size()));
            std::sort(blockers.begin(), blockers.end());
            stats.local_repair_robots = {critical};
            stats.local_repair_robots.insert(stats.local_repair_robots.end(), blockers.begin(), blockers.end());
            stats.local_repair_attempted = true;
            stats.local_repair_makespan_before_ticks = plans[critical].cost;
            stats.local_repair_makespan_after_ticks = plans[critical].cost;
            const auto started = std::chrono::steady_clock::now();
            const auto check = [&]()
            {
                if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >=
                    options.coordination_repair_budget_seconds)
                    throw RepairBudgetReached{};
            };
            CoordinationStats trial_stats;
            environment.set_stats(&trial_stats);
            environment.set_budget(check);
            try
            {
                std::vector<Plan> candidate;
                long long original_total = 0;
                for (const auto &plan : plans)
                {
                    check();
                    original_total += plan.cost;
                    candidate.push_back(plan);
                }
                std::vector<std::size_t> fixed;
                for (std::size_t r = 0; r < plans.size(); ++r)
                    if (std::find(stats.local_repair_robots.begin(), stats.local_repair_robots.end(), r) ==
                        stats.local_repair_robots.end()) fixed.push_back(r);
                bool complete = true;
                for (const auto agent : stats.local_repair_robots)
                {
                    check();
                    if (trial_stats.safe_interval_expanded_nodes >= options.coordination_repair_max_expanded_nodes)
                        throw RepairBudgetReached{};
                    environment.prepare_reservations(agent, candidate, fixed, horizon);
                    Constraints constraints;
                    environment.setLowLevelContext(agent, &constraints);
                    std::vector<std::size_t> frames;
                    if (!schedule_safe_intervals(environment.reservations(), environment.frames(agent), agent,
                                                 frames, &trial_stats,
                                                 options.coordination_repair_max_expanded_nodes -
                                                 trial_stats.safe_interval_expanded_nodes))
                    {
                        stats.local_repair_budget_exhausted =
                            trial_stats.safe_interval_expanded_nodes >= options.coordination_repair_max_expanded_nodes;
                        complete = false;
                        break;
                    }
                    Plan plan;
                    for (std::size_t tick = 0; tick < frames.size(); ++tick)
                    {
                        if ((tick & 255U) == 0U) check();
                        plan.states.push_back({{static_cast<int>(tick), frames[tick]}, static_cast<int>(tick)});
                    }
                    plan.cost = plan.fmin = static_cast<int>(frames.size() - 1U);
                    candidate[agent] = std::move(plan);
                    fixed.push_back(agent);
                }
                if (complete)
                {
                    int makespan = 0;
                    long long total = 0;
                    for (const auto &plan : candidate)
                    {
                        makespan = std::max(makespan, plan.cost);
                        total += plan.cost;
                    }
                    Conflict conflict;
                    if (makespan < stats.local_repair_makespan_before_ticks && total <= original_total &&
                        !environment.getFirstConflict(candidate, conflict))
                    {
                        check();
                        plans.swap(candidate);
                        stats.local_repair_accepted = true;
                        stats.local_repair_makespan_after_ticks = makespan;
                    }
                }
            }
            catch (const RepairBudgetReached &)
            {
                stats.local_repair_budget_exhausted = true;
            }
            catch (...)
            {
                environment.set_budget({});
                environment.set_stats(&stats);
                throw;
            }
            environment.set_budget({});
            environment.set_stats(&stats);
            stats.local_repair_expanded_nodes = trial_stats.safe_interval_expanded_nodes;
            stats.local_repair_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        }
    } // namespace

    std::vector<std::vector<TimedMapState>> coordinate_multi_map_routes(
        const MultiMapPathPlanner &path_planner,
        const std::vector<MappedRobot> &robots,
        const std::vector<MappedRobotRoute> &routes,
        CoordinationStats *stats)
    {
        if (robots.size() != routes.size())
            throw std::invalid_argument("robot and route counts must match");
        std::vector<std::vector<RouteFrame>> frames;
        std::vector<State> starts;
        std::size_t maximum_tick = 0U;
        frames.reserve(robots.size());
        starts.reserve(robots.size());
        for (std::size_t i = 0; i < robots.size(); ++i)
        {
            frames.push_back(make_frames(robots[i], routes[i], path_planner.options()));
            maximum_tick += frames.back().size();
            starts.push_back(State{0, 0U});
        }

        CoordinationStats internal_stats;
        if (!stats) stats = &internal_stats;
        *stats = {};
        Environment environment(std::move(frames), path_planner.bundle(), robots, stats);
        std::vector<Plan> initial_plans(robots.size());
        for (auto &plan : initial_plans) plan.states.push_back({{0, 0}, 0});
        Conflict initial_conflict;
        if (environment.getFirstConflict(initial_plans, initial_conflict))
            throw std::runtime_error("robot starting positions or resources already conflict at tick zero");
        std::vector<Plan> plans;
        if (stats)
            stats->prioritized_attempted = true;
        const bool prioritized_succeeded = prioritized_schedule(environment, starts,
                                                                static_cast<int>(std::min(maximum_tick,
                                                                                          static_cast<std::size_t>(std::numeric_limits<int>::max() - 1))),
                                                                plans);
        if (stats)
            stats->prioritized_succeeded = prioritized_succeeded;
        if (!prioritized_succeeded)
        {
            if (stats)
                stats->cbs_started = true;
            environment.set_cbs_phase(true);
            search::CBS<State, Action, int, Conflict, Constraints, Environment, StateHash> cbs(
                environment, path_planner.options().coordination_max_high_level_nodes);
            if (!cbs.search(starts, plans))
            {
                if (cbs.limit_reached())
                    throw std::runtime_error("multi-map CBS reached its high-level node limit");
                throw std::runtime_error("multi-map CBS could not find a conflict-free schedule");
            }
            if (stats)
                stats->cbs_succeeded = true;
        }
        else
            repair_critical_schedule(environment, path_planner.options(),
                                     static_cast<int>(std::min(maximum_tick,
                                         static_cast<std::size_t>(std::numeric_limits<int>::max() - 1))), plans, *stats);

        std::vector<std::vector<TimedMapState>> output(plans.size());
        for (std::size_t i = 0; i < plans.size(); ++i)
        {
            output[i].reserve(plans[i].states.size());
            for (const auto &[state, cost] : plans[i].states)
            {
                (void)cost;
                std::string transition;
                constexpr const char prefix[] = "transition:";
                const auto &frame = environment.route_frame(i, state);
                if (frame.resource.compare(0, sizeof(prefix) - 1U, prefix) == 0)
                    transition = frame.resource.substr(sizeof(prefix) - 1U);
                output[i].push_back({frame.position, state.tick, std::move(transition),
                                     state.frame, frame.resource});
            }
            for (std::size_t frame = 1; frame < plans[i].states.size(); ++frame)
            {
                const auto &previous = plans[i].states[frame - 1U].first;
                const auto &current = plans[i].states[frame].first;
                if (current.frame == previous.frame && stats)
                    ++stats->total_wait_ticks;
            }
        }
        return output;
    }
} // namespace capability_mission_planner::offline
