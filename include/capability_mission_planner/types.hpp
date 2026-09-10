#pragma once

#include <capability_mission_planner/task_metadata.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace capability_mission_planner
{
    using Capability = std::string;
    using CapabilitySet = std::set<Capability>;

    struct Location
    {
        int x = 0;
        int y = 0;

        bool operator==(const Location &other) const
        {
            return x == other.x && y == other.y;
        }

        bool operator!=(const Location &other) const
        {
            return !(*this == other);
        }

        bool operator<(const Location &other) const
        {
            return x < other.x || (x == other.x && y < other.y);
        }
    };

    struct LocationHash
    {
        std::size_t operator()(const Location &value) const
        {
            const auto x = static_cast<std::size_t>(static_cast<unsigned int>(value.x));
            const auto y = static_cast<std::size_t>(static_cast<unsigned int>(value.y));
            return (x << 32U) ^ y;
        }
    };

    struct Robot
    {
        std::string id;
        Location start;
        CapabilitySet capabilities;
        bool return_home = true;
    };

    struct AtomicTask
    {
        ConstTaskBookingPtr booking;
        TaskHeader header;
        Location location;
        CapabilitySet requirements;

        const std::string &id() const { return booking->id(); }

        int service_duration() const
        {
            const auto seconds = std::chrono::duration<double>(header.original_duration_estimate()).count();
            return std::max(0, static_cast<int>(std::ceil(seconds)));
        }

        bool high_priority() const
        {
            return booking->priority() == TaskPriority::High;
        }
    };

    inline AtomicTask make_task(
        std::string id,
        Location location,
        CapabilitySet requirements,
        std::string category,
        int service_seconds,
        bool high_priority = false,
        int earliest_start_seconds = 0)
    {
        const auto priority = high_priority ? TaskPriority::High : TaskPriority::Normal;
        const auto start = TaskTime{} + std::chrono::seconds(earliest_start_seconds);
        auto booking = std::make_shared<TaskBooking>(std::move(id), start, priority, false, std::vector<std::string>{category});
        TaskHeader header(std::move(category), "capability-aware atomic task", std::chrono::seconds(service_seconds));
        return AtomicTask{std::move(booking), std::move(header), location, std::move(requirements)};
    }

    inline bool is_compatible(const Robot &robot, const AtomicTask &task)
    {
        for (const auto &required : task.requirements)
        {
            if (robot.capabilities.count(required) == 0U)
                return false;
        }
        return true;
    }

    struct RouteStop
    {
        Location location;
        std::vector<std::size_t> task_indices;
        int service_duration = 0;
    };

    struct RobotRoute
    {
        std::size_t robot_index = 0;
        std::vector<RouteStop> stops;
        int travel_cost = 0;
        int service_cost = 0;

        int load() const { return travel_cost + service_cost; }
    };

    struct TimedState
    {
        Location location;
        int time = 0;
    };

    struct MissionPlan
    {
        std::vector<RobotRoute> routes;
        std::vector<std::vector<TimedState>> schedules;
        int maximum_load = 0;
        int total_load = 0;
    };
} // namespace capability_mission_planner
