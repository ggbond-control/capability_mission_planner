#pragma once

#include <capability_mission_planner/offline_map_planner.hpp>

namespace capability_mission_planner::offline
{
    inline const char *coordination_method(const OfflineMissionPlan &plan)
    {
        if (plan.coordination_stats.cbs_succeeded)
            return "cbs";
        if (plan.coordination_stats.prioritized_succeeded)
            return "prioritized";
        return "none";
    }

    void validate_animation(const std::vector<MappedRobot> &robots,
                            const OfflineMissionPlan &plan, const ExportOptions &options);
    void write_animation(const std::filesystem::path &path, const MultiMapBundle &bundle,
                         const std::vector<MappedRobot> &robots, const std::vector<MappedTask> &tasks,
                         const OfflineMissionPlan &plan,
                         const ExportOptions &options);
}
