#include <capability_mission_planner/grid_path_planner.hpp>

#include <capability_mission_planner/search/a_star.hpp>

#include <array>
#include <cstdlib>
#include <stdexcept>

namespace capability_mission_planner
{
    namespace
    {
        struct GridEnvironment
        {
            const GridMap &map;
            Location goal;

            int admissibleHeuristic(const Location &state) const
            {
                return std::abs(state.x - goal.x) + std::abs(state.y - goal.y);
            }

            bool isSolution(const Location &state) const { return state == goal; }

            void getNeighbors(const Location &state, std::vector<search::Neighbor<Location, Location, int>> &neighbors) const
            {
                static constexpr std::array<Location, 4> offsets{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
                for (const auto &offset : offsets)
                {
                    const Location next{state.x + offset.x, state.y + offset.y};
                    if (map.traversable(next))
                        neighbors.emplace_back(next, next, 1);
                }
            }

            void onExpandNode(const Location &, int, int) const {}
            void onDiscover(const Location &, int, int) const {}
        };
    } // namespace

    GridMap::GridMap(int width, int height, std::unordered_set<Location, LocationHash> obstacles)
        : _width(width), _height(height), _obstacles(std::move(obstacles))
    {
        if (width <= 0 || height <= 0)
            throw std::invalid_argument("grid dimensions must be positive");
    }

    bool GridMap::traversable(const Location &location) const
    {
        return location.x >= 0 && location.x < _width &&
               location.y >= 0 && location.y < _height &&
               _obstacles.count(location) == 0U;
    }

    GridPathPlanner::GridPathPlanner(GridMap map)
        : _map(std::move(map))
    {
    }

    SegmentPath GridPathPlanner::plan(const Location &start, const Location &goal) const
    {
        if (!_map.traversable(start) || !_map.traversable(goal))
            throw std::invalid_argument("path endpoint is outside the traversable grid");

        GridEnvironment environment{_map, goal};
        search::AStar<Location, Location, int, GridEnvironment, LocationHash> search(environment);
        capability_mission_planner::search::PlanResult<Location, Location, int> result;
        if (!search.search(start, result))
            throw std::runtime_error("no path between requested locations");

        SegmentPath output;
        output.cost = result.cost;
        output.states.reserve(result.states.size());
        for (const auto &state : result.states)
            output.states.push_back(state.first);
        return output;
    }

    int GridPathPlanner::distance(const Location &start, const Location &goal) const
    {
        return plan(start, goal).cost;
    }
} // namespace capability_mission_planner
