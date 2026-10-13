#pragma once

#include "route_reservations.hpp"

namespace capability_mission_planner::offline
{
    bool schedule_safe_intervals(RouteReservations &reservations, const std::vector<RouteFrame> &route,
                                 std::size_t robot, std::vector<std::size_t> &frames,
                                 CoordinationStats *stats, std::size_t node_limit = 0U);
}
