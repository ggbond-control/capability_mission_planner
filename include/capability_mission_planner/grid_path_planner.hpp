#pragma once

#include <capability_mission_planner/types.hpp>

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace capability_mission_planner
{
    class GridMap
    {
    public:
        GridMap(int width, int height, std::unordered_set<Location, LocationHash> obstacles = {});

        int width() const { return _width; }
        int height() const { return _height; }
        bool traversable(const Location &location) const;

    private:
        int _width;
        int _height;
        std::unordered_set<Location, LocationHash> _obstacles;
    };

    struct SegmentPath
    {
        std::vector<Location> states;
        int cost = 0;
    };

    class GridPathPlanner
    {
    public:
        explicit GridPathPlanner(GridMap map);

        SegmentPath plan(const Location &start, const Location &goal) const;
        int distance(const Location &start, const Location &goal) const;
        const GridMap &map() const { return _map; }

    private:
        GridMap _map;
    };
} // namespace capability_mission_planner
