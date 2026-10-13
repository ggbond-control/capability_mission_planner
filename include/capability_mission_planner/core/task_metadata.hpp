/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Adapted from rmf_task metadata types under the Apache License, Version 2.0.
 * See third_party/licenses/rmf_task-Apache-2.0.txt.
 */
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace capability_mission_planner
{
    using TaskTime = std::chrono::steady_clock::time_point;
    using TaskDuration = std::chrono::steady_clock::duration;

    enum class TaskPriority
    {
        Normal,
        High
    };

    class TaskBooking
    {
    public:
        TaskBooking(
            std::string id,
            TaskTime earliest_start_time,
            TaskPriority priority,
            bool automatic = false,
            std::vector<std::string> labels = {});

        const std::string &id() const;
        TaskTime earliest_start_time() const;
        TaskPriority priority() const;
        bool automatic() const;
        const std::vector<std::string> &labels() const;

    private:
        std::string _id;
        TaskTime _earliest_start_time;
        TaskPriority _priority;
        bool _automatic;
        std::vector<std::string> _labels;
    };

    using ConstTaskBookingPtr = std::shared_ptr<const TaskBooking>;

    class TaskHeader
    {
    public:
        TaskHeader(std::string category, std::string detail, TaskDuration estimate);

        const std::string &category() const;
        const std::string &detail() const;
        TaskDuration original_duration_estimate() const;

    private:
        std::string _category;
        std::string _detail;
        TaskDuration _duration;
    };
} // namespace capability_mission_planner
