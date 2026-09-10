#pragma once

#include <string>
#include <vector>

namespace capability_mission_planner::offline
{
    struct ResourceReservation
    {
        std::string resource;
        std::string robot_id;
        int start_tick = 0;
        int end_tick = 0;
    };

    // Execution-layer reservation table. It is intentionally independent of ROS2:
    // an adapter can call update_progress() whenever Nav2 reports new progress.
    class LiveReservationTable
    {
    public:
        bool try_reserve(const ResourceReservation &reservation);
        void release_robot(const std::string &robot_id);
        void update_progress(const std::string &robot_id, int current_tick);
        const std::vector<ResourceReservation> &reservations() const { return _reservations; }

    private:
        std::vector<ResourceReservation> _reservations;
    };
} // namespace capability_mission_planner::offline
