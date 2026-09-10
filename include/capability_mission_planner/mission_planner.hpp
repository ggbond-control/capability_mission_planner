#pragma once

#include <capability_mission_planner/grid_path_planner.hpp>

namespace capability_mission_planner
{
    struct ObjectiveWeights
    {
        double maximum_load = 1.0;
        double total_load = 0.1;
    };

    class MissionPlanner
    {
    public:
        MissionPlanner(GridPathPlanner path_planner, ObjectiveWeights weights = {});

        MissionPlan plan(
            const std::vector<Robot> &robots,
            const std::vector<AtomicTask> &tasks,
            bool coordinate_conflicts = true) const;

    private:
        GridPathPlanner _path_planner;
        ObjectiveWeights _weights;
    };
} // namespace capability_mission_planner
