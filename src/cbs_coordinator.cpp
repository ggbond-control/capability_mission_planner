#include <capability_mission_planner/cbs_coordinator.hpp>

#include <capability_mission_planner/search/conflict_based_search.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <map>
#include <unordered_set>

namespace capability_mission_planner
{
    struct CbsState
    {
        int time = 0;
        Location location;
        std::size_t progress = 0;

        bool operator==(const CbsState &other) const
        {
            return time == other.time && location == other.location && progress == other.progress;
        }
    };
} // namespace capability_mission_planner

namespace std
{
    template <>
    struct hash<capability_mission_planner::CbsState>
    {
        std::size_t operator()(const capability_mission_planner::CbsState &state) const noexcept
        {
            return capability_mission_planner::LocationHash{}(state.location) ^
                   (static_cast<std::size_t>(state.time) * 0x9e3779b9U) ^
                   (state.progress * 0x85ebca6bU);
        }
    };
} // namespace std

namespace capability_mission_planner
{
    namespace
    {
        using State = CbsState;

        enum class Action
        {
            Wait,
            Move
        };

        struct VertexConstraint
        {
            int time;
            Location location;

            bool operator==(const VertexConstraint &other) const
            {
                return time == other.time && location == other.location;
            }
        };

        struct VertexConstraintHash
        {
            std::size_t operator()(const VertexConstraint &value) const
            {
                return LocationHash{}(value.location) ^
                       (static_cast<std::size_t>(value.time) * 0x9e3779b9U);
            }
        };

        struct EdgeConstraint
        {
            int time;
            Location from;
            Location to;

            bool operator==(const EdgeConstraint &other) const
            {
                return time == other.time && from == other.from && to == other.to;
            }
        };

        struct EdgeConstraintHash
        {
            std::size_t operator()(const EdgeConstraint &value) const
            {
                return LocationHash{}(value.from) ^
                       (LocationHash{}(value.to) << 1U) ^
                       (static_cast<std::size_t>(value.time) * 0x9e3779b9U);
            }
        };

        struct Constraints
        {
            std::unordered_set<VertexConstraint, VertexConstraintHash> vertex;
            std::unordered_set<EdgeConstraint, EdgeConstraintHash> edge;

            void add(const Constraints &other)
            {
                vertex.insert(other.vertex.begin(), other.vertex.end());
                edge.insert(other.edge.begin(), other.edge.end());
            }

            bool overlap(const Constraints &other) const
            {
                for (const auto &constraint : other.vertex)
                {
                    if (vertex.count(constraint) != 0U)
                        return true;
                }
                for (const auto &constraint : other.edge)
                {
                    if (edge.count(constraint) != 0U)
                        return true;
                }
                return false;
            }
        };

        struct Conflict
        {
            enum class Type
            {
                Vertex,
                Edge
            } type = Type::Vertex;
            int time = 0;
            std::size_t first = 0;
            std::size_t second = 0;
            Location first_from;
            Location first_to;
            Location second_from;
            Location second_to;
        };

        using Plan = search::PlanResult<State, Action, int>;

        State state_at(const Plan &plan, std::size_t time)
        {
            if (time < plan.states.size())
                return plan.states[time].first;
            auto state = plan.states.back().first;
            state.time = static_cast<int>(time);
            return state;
        }

        class Environment
        {
        public:
            Environment(
                const GridMap &map,
                std::vector<std::vector<Location>> milestones,
                std::vector<Location> goals)
                : _map(map),
                  _milestones(std::move(milestones)),
                  _goals(std::move(goals))
            {
            }

            void setLowLevelContext(std::size_t agent, const Constraints *constraints)
            {
                _agent = agent;
                _constraints = constraints;
                _last_goal_constraint = -1;
                const auto goal = _goals[agent];
                for (const auto &constraint : constraints->vertex)
                {
                    if (constraint.location == goal)
                        _last_goal_constraint = std::max(_last_goal_constraint, constraint.time);
                }
            }

            int admissibleHeuristic(const State &state) const
            {
                if (_milestones[_agent].empty())
                {
                    return std::abs(state.location.x - _goals[_agent].x) +
                           std::abs(state.location.y - _goals[_agent].y);
                }
                int result = 0;
                Location current = state.location;
                for (std::size_t i = state.progress; i < _milestones[_agent].size(); ++i)
                {
                    const auto &next = _milestones[_agent][i];
                    result += std::abs(current.x - next.x) + std::abs(current.y - next.y);
                    current = next;
                }
                return result;
            }

            bool isSolution(const State &state) const
            {
                return state.progress == _milestones[_agent].size() &&
                       state.location == _goals[_agent] &&
                       state.time > _last_goal_constraint;
            }

            void getNeighbors(const State &state, std::vector<search::Neighbor<State, Action, int>> &neighbors) const
            {
                static constexpr std::array<Location, 5> offsets{{{0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
                for (const auto &offset : offsets)
                {
                    const Location next_location{state.location.x + offset.x, state.location.y + offset.y};
                    if (!_map.traversable(next_location))
                        continue;

                    State next{state.time + 1, next_location, state.progress};
                    if (next.progress < _milestones[_agent].size() && next.location == _milestones[_agent][next.progress])
                    {
                        ++next.progress;
                    }

                    if (_constraints->vertex.count(VertexConstraint{next.time, next.location}) != 0U)
                    {
                        continue;
                    }
                    if (_constraints->edge.count(EdgeConstraint{state.time, state.location, next.location}) != 0U)
                    {
                        continue;
                    }
                    neighbors.emplace_back(next, offset == Location{0, 0} ? Action::Wait : Action::Move, 1);
                }
            }

            bool getFirstConflict(const std::vector<Plan> &plans, Conflict &output) const
            {
                std::size_t horizon = 0;
                for (const auto &plan : plans)
                    horizon = std::max(horizon, plan.states.size());

                for (std::size_t time = 0; time < horizon; ++time)
                {
                    for (std::size_t a = 0; a < plans.size(); ++a)
                    {
                        for (std::size_t b = a + 1; b < plans.size(); ++b)
                        {
                            const auto a_now = state_at(plans[a], time);
                            const auto b_now = state_at(plans[b], time);
                            if (a_now.location == b_now.location)
                            {
                                output = Conflict{
                                    Conflict::Type::Vertex, static_cast<int>(time), a, b,
                                    a_now.location, a_now.location, b_now.location, b_now.location};
                                return true;
                            }
                            if (time + 1 < horizon)
                            {
                                const auto a_next = state_at(plans[a], time + 1);
                                const auto b_next = state_at(plans[b], time + 1);
                                if (a_now.location == b_next.location && b_now.location == a_next.location)
                                {
                                    output = Conflict{
                                        Conflict::Type::Edge, static_cast<int>(time), a, b,
                                        a_now.location, a_next.location,
                                        b_now.location, b_next.location};
                                    return true;
                                }
                            }
                        }
                    }
                }
                return false;
            }

            void createConstraintsFromConflict(const Conflict &conflict, std::map<std::size_t, Constraints> &output) const
            {
                if (conflict.type == Conflict::Type::Vertex)
                {
                    Constraints constraint;
                    constraint.vertex.insert(VertexConstraint{conflict.time, conflict.first_to});
                    output[conflict.first] = constraint;
                    output[conflict.second] = constraint;
                    return;
                }

                Constraints first;
                first.edge.insert(EdgeConstraint{conflict.time, conflict.first_from, conflict.first_to});
                output[conflict.first] = first;
                Constraints second;
                second.edge.insert(EdgeConstraint{conflict.time, conflict.second_from, conflict.second_to});
                output[conflict.second] = second;
            }

            void onExpandHighLevelNode(int) {}
            void onExpandLowLevelNode(const State &, int, int) {}

        private:
            const GridMap &_map;
            std::vector<std::vector<Location>> _milestones;
            std::vector<Location> _goals;
            std::size_t _agent = 0;
            const Constraints *_constraints = nullptr;
            int _last_goal_constraint = -1;
        };

        std::vector<Location> make_milestones(const Robot &robot, const RobotRoute &route)
        {
            std::vector<Location> result;
            for (const auto &stop : route.stops)
            {
                result.push_back(stop.location);
                for (int i = 0; i < stop.service_duration; ++i)
                    result.push_back(stop.location);
            }
            if (robot.return_home && !route.stops.empty() && route.stops.back().location != robot.start)
            {
                result.push_back(robot.start);
            }
            return result;
        }
    } // namespace

    CbsCoordinator::CbsCoordinator(const GridMap &map) : _map(map)
    {
    }

    std::vector<std::vector<TimedState>> CbsCoordinator::coordinate(const std::vector<Robot> &robots, const std::vector<RobotRoute> &routes) const
    {
        if (robots.size() != routes.size())
            throw std::invalid_argument("one route is required for each robot");

        std::vector<std::vector<Location>> milestones;
        std::vector<Location> goals;
        std::vector<State> starts;
        milestones.reserve(robots.size());
        goals.reserve(robots.size());
        starts.reserve(robots.size());
        std::unordered_set<Location, LocationHash> unique_starts;
        for (std::size_t i = 0; i < robots.size(); ++i)
        {
            if (!unique_starts.insert(robots[i].start).second)
                throw std::invalid_argument("CBS requires unique initial robot locations");
            milestones.push_back(make_milestones(robots[i], routes[i]));
            goals.push_back(milestones.back().empty()
                                ? robots[i].start
                                : milestones.back().back());
            std::size_t progress = 0;
            if (!milestones.back().empty() && milestones.back().front() == robots[i].start)
                progress = 1;
            starts.push_back(State{0, robots[i].start, progress});
        }

        Environment environment(_map, std::move(milestones), std::move(goals));
        search::CBS<State, Action, int, Conflict, Constraints, Environment> cbs(environment);
        std::vector<Plan> plans;
        const bool success = cbs.search(starts, plans);
        if (!success)
            throw std::runtime_error("CBS could not find a conflict-free mission schedule");

        std::vector<std::vector<TimedState>> output(plans.size());
        for (std::size_t i = 0; i < plans.size(); ++i)
        {
            output[i].reserve(plans[i].states.size());
            for (const auto &state : plans[i].states)
                output[i].push_back(TimedState{state.first.location, state.first.time});
        }
        return output;
    }
} // namespace capability_mission_planner
