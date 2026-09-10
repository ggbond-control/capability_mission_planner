#include <capability_mission_planner/live_reservation.hpp>

#include <algorithm>

namespace capability_mission_planner::offline
{
    bool LiveReservationTable::try_reserve(const ResourceReservation &candidate)
    {
        if (candidate.resource.empty() || candidate.robot_id.empty() || candidate.start_tick > candidate.end_tick)
            return false;
        for (const auto &existing : _reservations)
        {
            if (existing.resource == candidate.resource && existing.robot_id != candidate.robot_id &&
                existing.start_tick <= candidate.end_tick && candidate.start_tick <= existing.end_tick)
                return false;
        }
        _reservations.erase(std::remove_if(_reservations.begin(), _reservations.end(),
                                           [&](const auto &item)
                                           { return item.robot_id == candidate.robot_id && item.resource == candidate.resource; }),
                            _reservations.end());
        _reservations.push_back(candidate);
        return true;
    }

    void LiveReservationTable::release_robot(const std::string &robot_id)
    {
        _reservations.erase(std::remove_if(_reservations.begin(), _reservations.end(),
                                           [&](const auto &item)
                                           { return item.robot_id == robot_id; }),
                            _reservations.end());
    }

    void LiveReservationTable::update_progress(const std::string &robot_id, int current_tick)
    {
        for (auto &item : _reservations)
        {
            if (item.robot_id == robot_id)
                item.start_tick = std::max(item.start_tick, current_tick);
        }
        _reservations.erase(std::remove_if(_reservations.begin(), _reservations.end(),
                                           [&](const auto &item)
                                           { return item.robot_id == robot_id && item.end_tick < current_tick; }),
                            _reservations.end());
    }
} // namespace capability_mission_planner::offline
