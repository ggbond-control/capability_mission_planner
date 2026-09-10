/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Adapted from rmf_task metadata types under the Apache License, Version 2.0.
 * See third_party/licenses/rmf_task-Apache-2.0.txt.
 */
#include <capability_mission_planner/task_metadata.hpp>

#include <utility>

namespace capability_mission_planner
{
    TaskBooking::TaskBooking(
        std::string id,
        TaskTime earliest_start_time,
        TaskPriority priority,
        bool automatic,
        std::vector<std::string> labels)
        : _id(std::move(id)),
          _earliest_start_time(earliest_start_time),
          _priority(priority),
          _automatic(automatic),
          _labels(std::move(labels))
    {
    }

    const std::string &TaskBooking::id() const { return _id; }
    TaskTime TaskBooking::earliest_start_time() const { return _earliest_start_time; }
    TaskPriority TaskBooking::priority() const { return _priority; }
    bool TaskBooking::automatic() const { return _automatic; }
    const std::vector<std::string> &TaskBooking::labels() const { return _labels; }

    TaskHeader::TaskHeader(
        std::string category,
        std::string detail,
        TaskDuration estimate)
        : _category(std::move(category)),
          _detail(std::move(detail)),
          _duration(estimate)
    {
    }

    const std::string &TaskHeader::category() const { return _category; }
    const std::string &TaskHeader::detail() const { return _detail; }
    TaskDuration TaskHeader::original_duration_estimate() const { return _duration; }

} // namespace capability_mission_planner
