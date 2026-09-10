#pragma once

#include <capability_mission_planner/grid_path_planner.hpp>

namespace capability_mission_planner
{
    class CbsCoordinator
    {
    public:
        explicit CbsCoordinator(const GridMap &map);

        std::vector<std::vector<TimedState>> coordinate(const std::vector<Robot> &robots, const std::vector<RobotRoute> &routes) const;

    private:
        const GridMap &_map;
    };
} // namespace capability_mission_planner
