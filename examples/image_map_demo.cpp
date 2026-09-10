#include <capability_mission_planner/mission_planner.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using namespace capability_mission_planner;

namespace
{
    constexpr int cell_size = 5;

    struct RasterizedMap
    {
        GridMap map;
        int grid_width;
        int grid_height;
        std::vector<unsigned char> free_cells;
    };

    std::size_t index_of(int x, int y, int width)
    {
        return static_cast<std::size_t>(y * width + x);
    }

    RasterizedMap rasterize(const cv::Mat &image)
    {
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        const int width = (gray.cols + cell_size - 1) / cell_size;
        const int height = (gray.rows + cell_size - 1) / cell_size;
        std::vector<unsigned char> candidate(static_cast<std::size_t>(width * height));

        for (int gy = 0; gy < height; ++gy)
        {
            for (int gx = 0; gx < width; ++gx)
            {
                const int x0 = gx * cell_size;
                const int y0 = gy * cell_size;
                const int x1 = std::min(x0 + cell_size, gray.cols);
                const int y1 = std::min(y0 + cell_size, gray.rows);
                int white = 0;
                int black = 0;
                const int pixels = (x1 - x0) * (y1 - y0);
                for (int y = y0; y < y1; ++y)
                {
                    for (int x = x0; x < x1; ++x)
                    {
                        const auto value = gray.at<unsigned char>(y, x);
                        white += value >= 245;
                        black += value <= 50;
                    }
                }
                const double white_ratio = static_cast<double>(white) / pixels;
                const double black_ratio = static_cast<double>(black) / pixels;
                candidate[index_of(gx, gy, width)] = white_ratio >= 0.55 && black_ratio < 0.12;
            }
        }

        // Retain the main connected floor area. This removes scan speckles and
        // disconnected white regions outside the building footprint.
        std::vector<int> component(candidate.size(), -1);
        std::vector<int> sizes;
        static constexpr int offsets[4][2]{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const auto seed = index_of(x, y, width);
                if (!candidate[seed] || component[seed] >= 0)
                    continue;
                const int id = static_cast<int>(sizes.size());
                int size = 0;
                std::queue<Location> queue;
                queue.push({x, y});
                component[seed] = id;
                while (!queue.empty())
                {
                    const auto current = queue.front();
                    queue.pop();
                    ++size;
                    for (const auto &offset : offsets)
                    {
                        const int nx = current.x + offset[0];
                        const int ny = current.y + offset[1];
                        if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                            continue;
                        const auto next = index_of(nx, ny, width);
                        if (candidate[next] && component[next] < 0)
                        {
                            component[next] = id;
                            queue.push({nx, ny});
                        }
                    }
                }
                sizes.push_back(size);
            }
        }
        if (sizes.empty())
            throw std::runtime_error("image contains no connected traversable area");
        const int main_component = static_cast<int>(std::distance(sizes.begin(), std::max_element(sizes.begin(), sizes.end())));

        std::vector<unsigned char> free_cells(candidate.size());
        std::unordered_set<Location, LocationHash> obstacles;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const auto index = index_of(x, y, width);
                free_cells[index] = component[index] == main_component;
                if (!free_cells[index])
                    obstacles.insert({x, y});
            }
        }
        return {GridMap(width, height, std::move(obstacles)), width, height, std::move(free_cells)};
    }

    Location nearest_free(const RasterizedMap &raster, cv::Point desired, const std::set<Location> &used)
    {
        const Location target{desired.x / cell_size, desired.y / cell_size};
        int best_distance = std::numeric_limits<int>::max();
        Location best{-1, -1};
        for (int y = 1; y + 1 < raster.grid_height; ++y)
        {
            for (int x = 1; x + 1 < raster.grid_width; ++x)
            {
                const Location candidate{x, y};
                if (!raster.map.traversable(candidate) || used.count(candidate) != 0U)
                    continue;
                bool has_clearance = true;
                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dx = -1; dx <= 1; ++dx)
                        has_clearance = has_clearance && raster.map.traversable({x + dx, y + dy});
                }
                if (!has_clearance)
                    continue;
                const int distance = std::abs(x - target.x) + std::abs(y - target.y);
                if (distance < best_distance)
                {
                    best_distance = distance;
                    best = candidate;
                }
            }
        }
        if (best.x < 0)
            throw std::runtime_error("could not place a scenario point on free space");
        return best;
    }

    cv::Point pixel_center(const Location &location)
    {
        return {location.x * cell_size + cell_size / 2, location.y * cell_size + cell_size / 2};
    }

    void verify_plan(const RasterizedMap &raster, const std::vector<Robot> &robots, const std::vector<AtomicTask> &tasks, const MissionPlan &plan)
    {
        if (plan.routes.size() != robots.size() || plan.schedules.size() != robots.size())
            throw std::runtime_error("output does not contain one route and schedule per robot");

        std::set<std::size_t> assigned;
        for (const auto &route : plan.routes)
        {
            for (const auto &stop : route.stops)
            {
                if (!raster.map.traversable(stop.location))
                    throw std::runtime_error("a route stop is not traversable");
                for (const auto task : stop.task_indices)
                {
                    if (!assigned.insert(task).second)
                        throw std::runtime_error("a task was assigned more than once");
                    if (!is_compatible(robots[route.robot_index], tasks[task]))
                        throw std::runtime_error("a task was assigned to an incapable robot");
                }
            }
        }
        if (assigned.size() != tasks.size())
            throw std::runtime_error("not every task was assigned");

        std::size_t horizon = 0;
        for (const auto &schedule : plan.schedules)
        {
            if (schedule.empty())
                throw std::runtime_error("a robot schedule is empty");
            horizon = std::max(horizon, schedule.size());
            for (const auto &state : schedule)
            {
                if (!raster.map.traversable(state.location))
                    throw std::runtime_error("a timed state is not traversable");
            }
        }
        const auto at = [&plan](std::size_t robot, std::size_t time)
        {
            const auto &schedule = plan.schedules[robot];
            return schedule[std::min(time, schedule.size() - 1)].location;
        };
        for (std::size_t time = 0; time < horizon; ++time)
        {
            for (std::size_t a = 0; a < robots.size(); ++a)
            {
                for (std::size_t b = a + 1; b < robots.size(); ++b)
                {
                    if (at(a, time) == at(b, time))
                        throw std::runtime_error("CBS left a vertex conflict");
                    if (time + 1 < horizon && at(a, time) == at(b, time + 1) &&
                        at(b, time) == at(a, time + 1))
                    {
                        throw std::runtime_error("CBS left an opposing-edge conflict");
                    }
                }
            }
        }
    }

    cv::Mat render(const cv::Mat &source, const RasterizedMap &raster, const std::vector<Robot> &robots, const std::vector<AtomicTask> &tasks, const MissionPlan &plan)
    {
        constexpr int panel_width = 365;
        cv::Mat output(source.rows, source.cols + panel_width, CV_8UC3, cv::Scalar(248, 248, 248));
        source.copyTo(output(cv::Rect(0, 0, source.cols, source.rows)));

        const std::vector<cv::Scalar> colors{{35, 80, 220}, {40, 155, 35}, {210, 90, 30}, {170, 40, 160}};
        for (const auto &route : plan.routes)
        {
            const auto robot = route.robot_index;
            std::vector<cv::Point> points;
            for (const auto &state : plan.schedules[robot])
                points.push_back(pixel_center(state.location));
            if (points.size() > 1U)
                cv::polylines(output, points, false, colors[robot], 3, cv::LINE_AA);
        }

        std::vector<std::size_t> owner(tasks.size(), robots.size());
        for (const auto &route : plan.routes)
        {
            for (const auto &stop : route.stops)
            {
                for (const auto task : stop.task_indices)
                    owner[task] = route.robot_index;
            }
        }
        std::map<Location, std::vector<std::size_t>> tasks_by_location;
        for (std::size_t i = 0; i < tasks.size(); ++i)
            tasks_by_location[tasks[i].location].push_back(i);
        for (const auto &[location, indices] : tasks_by_location)
        {
            const auto point = pixel_center(location);
            const auto robot = owner[indices.front()];
            std::string marker;
            for (const auto index : indices)
            {
                if (!marker.empty())
                    marker += ',';
                marker += std::to_string(index + 1);
            }
            cv::circle(output, point, 7, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
            cv::circle(output, point, 6, colors[robot], 2, cv::LINE_AA);
            cv::putText(output, marker, point + cv::Point(8, -5), cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(20, 20, 20), 1, cv::LINE_AA);
        }
        for (std::size_t i = 0; i < robots.size(); ++i)
        {
            const auto point = pixel_center(robots[i].start);
            cv::circle(output, point, 9, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
            cv::circle(output, point, 8, colors[i], -1, cv::LINE_AA);
            cv::putText(output, robots[i].id, point + cv::Point(-4, 4), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }

        const int left = source.cols + 20;
        cv::putText(output, "CAPABILITY MISSION PLAN", {left, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(25, 25, 25), 2, cv::LINE_AA);
        cv::putText(output,
                    "source " + std::to_string(source.cols) + "x" +
                        std::to_string(source.rows) + "  |  grid " +
                        std::to_string(raster.grid_width) + "x" +
                        std::to_string(raster.grid_height) + "  |  cell 5px",
                    {left, 55}, cv::FONT_HERSHEY_SIMPLEX, 0.38,
                    cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
        cv::putText(output,
                    "max load " + std::to_string(plan.maximum_load) + "  |  total " + std::to_string(plan.total_load),
                    {left, 78}, cv::FONT_HERSHEY_SIMPLEX, 0.48,
                    cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
        cv::line(output,
                 {left, 92}, {output.cols - 20, 92},
                 cv::Scalar(205, 205, 205), 1);

        int y = 112;
        for (const auto &route : plan.routes)
        {
            const auto robot = route.robot_index;
            std::size_t task_count = 0;
            for (const auto &stop : route.stops)
                task_count += stop.task_indices.size();
            cv::circle(output, {left + 8, y - 5}, 6, colors[robot], -1, cv::LINE_AA);
            cv::putText(output,
                        "robot " + robots[robot].id + "  " + std::to_string(task_count) +
                            " tasks / " + std::to_string(route.stops.size()) + " stops",
                        {left + 23, y}, cv::FONT_HERSHEY_SIMPLEX, 0.46,
                        cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
            cv::putText(output,
                        "travel " + std::to_string(route.travel_cost) + " + service " +
                            std::to_string(route.service_cost) + " = load " +
                            std::to_string(route.load()),
                        {left + 23, y + 20}, cv::FONT_HERSHEY_SIMPLEX, 0.39,
                        cv::Scalar(85, 85, 85), 1, cv::LINE_AA);
            y += 50;
        }

        cv::line(output, {left, y - 15}, {output.cols - 20, y - 15}, cv::Scalar(205, 205, 205), 1);
        cv::putText(output, "TASKS (number = map marker)", {left, y + 8}, cv::FONT_HERSHEY_SIMPLEX, 0.43, cv::Scalar(45, 45, 45), 1, cv::LINE_AA);
        y += 26;
        for (std::size_t i = 0; i < tasks.size(); ++i)
        {
            const std::string label = std::to_string(i + 1) + "  " + tasks[i].id() + " -> " + robots[owner[i]].id;
            cv::putText(output, label, {left, y}, cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(55, 55, 55), 1, cv::LINE_AA);
            y += 15;
        }
        cv::putText(output, "PASS: assignment, paths and CBS conflicts", {left, output.rows - 18}, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(25, 120, 25), 1, cv::LINE_AA);
        return output;
    }
}

int main(int argc, char *argv[])
{
    try
    {
        const std::filesystem::path input = argc > 1 ? argv[1] : "examples/data/image.png";
        const std::filesystem::path output = argc > 2 ? argv[2] : "output/capability_mission_plan.png";
        const auto image = cv::imread(input.string(), cv::IMREAD_COLOR);
        if (image.empty())
            throw std::runtime_error("could not read input PNG: " + input.string());
        const auto raster = rasterize(image);

        std::set<Location> used;
        auto place = [&](cv::Point desired)
        {
            const auto location = nearest_free(raster, desired, used);
            used.insert(location);
            return location;
        };
        std::vector<Robot> robots{
            {"a", place({75, 370}), {"fire", "camera"}, true},
            {"b", place({175, 35}), {"camera", "thermal"}, true},
            {"c", place({275, 125}), {"fire", "camera", "thermal"}, true}};

        const auto p1 = place({75, 325});
        const auto p2 = place({120, 275});
        const auto p3 = place({185, 235});
        const auto p4 = place({245, 195});
        const auto p5 = place({250, 105});
        const auto p6 = place({175, 80});
        const auto p7 = place({135, 365});
        std::vector<AtomicTask> tasks{
            make_task("T1-photo", p1, {"camera"}, "gimbal_photo", 2, true),
            make_task("T2-fire", p1, {"fire"}, "fire_suppression", 4),
            make_task("T3-thermal", p2, {"thermal"}, "thermal_inspection", 3),
            make_task("T4-photo", p3, {"camera"}, "gimbal_photo", 2),
            make_task("T5-fire", p4, {"fire"}, "fire_suppression", 4, true),
            make_task("T6-thermal", p5, {"thermal"}, "thermal_inspection", 3),
            make_task("T7-photo", p6, {"camera"}, "gimbal_photo", 2),
            make_task("T8-fire-photo", p7, {"fire", "camera"}, "fire_documentation", 5)};

        MissionPlanner planner(GridPathPlanner(raster.map));
        const auto plan = planner.plan(robots, tasks, true);
        verify_plan(raster, robots, tasks, plan);
        if (!output.parent_path().empty())
            std::filesystem::create_directories(output.parent_path());
        if (!cv::imwrite(output.string(), render(image, raster, robots, tasks, plan)))
            throw std::runtime_error("could not write result PNG: " + output.string());

        std::cout << "PASS image=" << image.cols << 'x' << image.rows
                  << " grid=" << raster.grid_width << 'x' << raster.grid_height
                  << " robots=" << robots.size() << " tasks=" << tasks.size()
                  << " maximum_load=" << plan.maximum_load
                  << " total_load=" << plan.total_load
                  << " output=" << output.string() << '\n';
        for (const auto &route : plan.routes)
        {
            std::size_t count = 0;
            for (const auto &stop : route.stops)
                count += stop.task_indices.size();
            std::cout << "  " << robots[route.robot_index].id << ": tasks=" << count
                      << " stops=" << route.stops.size()
                      << " travel=" << route.travel_cost
                      << " service=" << route.service_cost
                      << " schedule=" << plan.schedules[route.robot_index].size()
                      << '\n';
        }
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "image map validation failed: " << error.what() << '\n';
        return 1;
    }
}
