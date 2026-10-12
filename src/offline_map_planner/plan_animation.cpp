#include "plan_animation.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>

#ifdef CMP_HAVE_GIF
#include <gif_lib.h>
#endif

namespace capability_mission_planner::offline
{
    void validate_animation(const std::vector<MappedRobot> &robots,
                            const OfflineMissionPlan &plan, const ExportOptions &options)
    {
        if (!options.animation)
            return;
#ifndef CMP_HAVE_GIF
        throw std::runtime_error("GIF export requires giflib: install libgif-dev and reconfigure CMake");
#endif
        if (options.animation_fps < 1 || options.animation_fps > 50 ||
            !std::isfinite(options.animation_speedup) || options.animation_speedup <= 0.0 ||
            options.animation_max_frames < 2 || options.animation_width < 640 ||
            options.animation_width > 4096 || !std::isfinite(plan.time_step_seconds) ||
            plan.time_step_seconds <= 0.0)
            throw std::invalid_argument("invalid GIF timing or dimensions");
        if (robots.empty() || plan.schedules.size() != robots.size())
            throw std::invalid_argument("GIF export requires timed schedules; enable planner.coordinate_conflicts");
        for (const auto &schedule : plan.schedules)
        {
            if (schedule.empty() || schedule.front().tick != 0)
                throw std::invalid_argument("GIF export requires nonempty schedules starting at tick 0");
            for (std::size_t i = 1; i < schedule.size(); ++i)
                if (schedule[i].tick <= schedule[i - 1U].tick)
                    throw std::invalid_argument("GIF schedule ticks must be strictly increasing");
        }
    }

#ifdef CMP_HAVE_GIF
    namespace
    {
        const cv::Scalar ink{55, 55, 55};
        const cv::Scalar amber{0, 160, 255};
        const std::vector<cv::Scalar> colors{
            {35, 75, 220}, {45, 155, 35}, {210, 95, 30},
            {165, 45, 170}, {30, 165, 190}, {120, 80, 35}};

        std::string seconds(double value)
        {
            std::ostringstream text;
            text << std::fixed << std::setprecision(1) << value;
            return text.str();
        }

        void label(cv::Mat &image, const std::string &text, cv::Point point,
                   const cv::Scalar &color = ink, double scale = 0.45)
        {
            cv::putText(image, text, point, cv::FONT_HERSHEY_SIMPLEX, scale,
                        color, 1, cv::LINE_AA);
        }

        struct Panel
        {
            cv::Rect bounds;
            double scale;
            int map_height;
            cv::Point pixel(const GridPosition &position) const
            {
                return {bounds.x + static_cast<int>(std::lround((position.x + 0.5) * scale)),
                        bounds.y + static_cast<int>(std::lround((map_height - position.y - 0.5) * scale))};
            }
        };

        // Fixed RGB cube plus a grayscale ramp keeps occupancy maps crisp and
        // avoids palette flickering. Encode one frame at a time, not the whole GIF.
        class GifWriter
        {
        public:
            GifWriter(const std::filesystem::path &path, int width, int height)
            {
                int error = 0;
                _file = EGifOpenFileName(path.string().c_str(), false, &error);
                if (!_file)
                    throw std::runtime_error("cannot open GIF: " + path.string());
                try
                {
                    std::array<GifColorType, 256> palette{};
                    for (int r = 0; r < 6; ++r)
                        for (int g = 0; g < 6; ++g)
                            for (int b = 0; b < 6; ++b)
                                palette[r * 36 + g * 6 + b] = {
                                    static_cast<GifByteType>(r * 51), static_cast<GifByteType>(g * 51),
                                    static_cast<GifByteType>(b * 51)};
                    // Index 255 is reserved for unchanged pixels in later frames.
                    for (int i = 0; i < 39; ++i)
                    {
                        const auto gray = static_cast<GifByteType>(std::lround(i * 255.0 / 38));
                        palette[216 + i] = {gray, gray, gray};
                    }
                    std::unique_ptr<ColorMapObject, decltype(&GifFreeMapObject)> map(
                        GifMakeMapObject(256, palette.data()), GifFreeMapObject);
                    if (!map)
                        throw std::runtime_error("cannot allocate GIF palette");
                    EGifSetGifVersion(_file, true);
                    check(EGifPutScreenDesc(_file, width, height, 8, 254, map.get()));
                    check(EGifPutExtensionLeader(_file, APPLICATION_EXT_FUNC_CODE));
                    check(EGifPutExtensionBlock(_file, 11, "NETSCAPE2.0"));
                    const unsigned char loop[] = {1, 0, 0};
                    check(EGifPutExtensionBlock(_file, 3, loop));
                    check(EGifPutExtensionTrailer(_file));
                }
                catch (...)
                {
                    EGifCloseFile(_file, &error);
                    _file = nullptr;
                    throw;
                }
            }
            ~GifWriter()
            {
                if (_file)
                {
                    int error = 0;
                    EGifCloseFile(_file, &error);
                }
            }
            void frame(const cv::Mat &image, int delay)
            {
                GraphicsControlBlock control{DISPOSE_DO_NOT, false, delay,
                                             _has_previous ? 255 : NO_TRANSPARENT_COLOR};
                GifByteType extension[4];
                EGifGCBToExtension(&control, extension);
                check(EGifPutExtension(_file, GRAPHICS_EXT_FUNC_CODE, 4, extension));
                check(EGifPutImageDesc(_file, 0, 0, image.cols, image.rows, false, nullptr));
                std::vector<GifPixelType> row(image.cols);
                if (!_has_previous)
                    _previous.resize(static_cast<std::size_t>(image.rows) * image.cols);
                for (int y = 0; y < image.rows; ++y)
                {
                    const auto *pixels = image.ptr<cv::Vec3b>(y);
                    for (int x = 0; x < image.cols; ++x)
                    {
                        const auto p = pixels[x];
                        GifPixelType value;
                        if (p[0] == p[1] && p[1] == p[2])
                            value = 216 + static_cast<int>(std::lround(p[0] * 38.0 / 255));
                        else
                            value = ((p[2] + 25) / 51) * 36 + ((p[1] + 25) / 51) * 6 + (p[0] + 25) / 51;
                        const auto offset = static_cast<std::size_t>(y) * image.cols + x;
                        row[x] = _has_previous && _previous[offset] == value ? 255 : value;
                        _previous[offset] = value;
                    }
                    check(EGifPutLine(_file, row.data(), image.cols));
                }
                _has_previous = true;
            }
            void close()
            {
                int error = 0;
                const auto result = EGifCloseFile(_file, &error);
                _file = nullptr;
                check(result);
            }
        private:
            static void check(int result)
            {
                if (result == GIF_ERROR)
                    throw std::runtime_error("GIF encoding failed");
            }
            GifFileType *_file = nullptr;
            bool _has_previous = false;
            std::vector<GifPixelType> _previous;
        };

        std::size_t state_at(const std::vector<TimedMapState> &schedule, int tick)
        {
            const auto next = std::upper_bound(schedule.begin(), schedule.end(), tick,
                [](int value, const auto &state) { return value < state.tick; });
            return static_cast<std::size_t>(std::distance(schedule.begin(), next) - 1);
        }
    }
#endif

    void write_animation(const std::filesystem::path &path, const MultiMapBundle &bundle,
                         const std::vector<MappedRobot> &robots, const std::vector<MappedTask> &tasks,
                         const OfflineMissionPlan &plan,
                         const ExportOptions &options)
    {
        validate_animation(robots, plan, options);
        if (!options.animation)
            return;
#ifdef CMP_HAVE_GIF
        if (bundle.maps.empty())
            throw std::invalid_argument("GIF export requires at least one map");
        const int sidebar = 300;
        const int map_width = options.animation_width - sidebar;
        const int columns = bundle.maps.size() > 1U ? 2 : 1;
        const int rows = static_cast<int>((bundle.maps.size() + columns - 1) / columns);
        const int tile_width = map_width / columns;
        const int tile_height = std::clamp(tile_width * bundle.maps.begin()->second.height /
                                         std::max(1, bundle.maps.begin()->second.width) + 30, 240,
                                         bundle.maps.size() == 1U ? 1100 : 600);
        const int height = std::max(110 + rows * tile_height,
                                   175 + static_cast<int>(robots.size()) * 90);
        if (height > 65535)
            throw std::invalid_argument("too many maps or robots for GIF canvas");
        cv::Mat base(height, options.animation_width, CV_8UC3, cv::Scalar(248, 248, 248));
        std::map<std::string, Panel> panels;
        int index = 0;
        for (const auto &[id, map] : bundle.maps)
        {
            const auto image = cv::imread(map.image_path.string(), cv::IMREAD_COLOR);
            if (image.empty())
                throw std::runtime_error("cannot render GIF map: " + map.image_path.string());
            const double scale = std::min((tile_width - 20.0) / map.width,
                                          (tile_height - 35.0) / map.height);
            const int width = std::max(1, static_cast<int>(map.width * scale));
            const int panel_height = std::max(1, static_cast<int>(map.height * scale));
            const int x = index % columns * tile_width + (tile_width - width) / 2;
            const int y = 100 + index / columns * tile_height + 25;
            const cv::Rect bounds{x, y, width, panel_height};
            cv::resize(image, base(bounds), bounds.size(), 0, 0, cv::INTER_AREA);
            label(base, "Map: " + id, {x, y - 8});
            panels.emplace(id, Panel{bounds, scale, map.height});
            ++index;
        }
        const auto point = [&](const GridPosition &p) { return panels.at(p.map_id).pixel(p); };
        // Full route is faint; the executed portion is drawn in the robot color.
        for (std::size_t r = 0; r < robots.size(); ++r)
        {
            const auto &schedule = plan.schedules[r];
            const auto color = colors[r % colors.size()];
            for (std::size_t i = 1; i < schedule.size(); ++i)
                if (schedule[i].position.map_id == schedule[i - 1].position.map_id &&
                    schedule[i].position != schedule[i - 1].position)
                    cv::line(base, point(schedule[i - 1].position), point(schedule[i].position),
                             color * 0.25 + cv::Scalar(190, 190, 190), 1, cv::LINE_AA);
            cv::drawMarker(base, point(robots[r].start), color, cv::MARKER_CROSS, 13, 2);
            if (r < plan.navigation_checkpoints.size())
                for (const auto &checkpoint : plan.navigation_checkpoints[r])
                    if (checkpoint.type == NavigationCheckpointType::Task)
                    {
                        const auto p = point(checkpoint.position);
                        cv::circle(base, p, 5, color, 1, cv::LINE_AA);
                        const auto task = std::find_if(tasks.begin(), tasks.end(), [&](const auto &value)
                                                      { return value.id() == checkpoint.task_id; });
                        if (task == tasks.end())
                            label(base, checkpoint.task_id.substr(0, 12), p + cv::Point(6, -6), ink, 0.32);
                    }
        }
        std::map<GridPosition, std::string> task_labels;
        for (std::size_t i = 0; i < tasks.size(); ++i)
        {
            auto &text = task_labels[tasks[i].location];
            if (!text.empty())
                text += '/';
            text += std::to_string(i + 1);
        }
        for (const auto &[position, text] : task_labels)
            label(base, text, point(position) + cv::Point(6, -6), ink, 0.32);
        label(base, "Synchronized mission replay | method: " + std::string(coordination_method(plan)),
              {15, 24}, ink, 0.58);
        label(base, "Colored trail: executed route   Amber ring: coordination WAIT   Square: task service   Tasks: input index", {15, 77});
        label(base, "ROBOTS / COORDINATION", {map_width + 12, 110});
        label(base, "Wait total: " + seconds(plan.coordination_stats.total_wait_ticks * plan.time_step_seconds) + " s",
              {map_width + 12, 132});

        int horizon = 0;
        for (const auto &schedule : plan.schedules)
            horizon = std::max(horizon, schedule.back().tick);
        // Frame limit increases stride, without modifying any planning state.
        const double desired_stride = options.animation_speedup / (options.animation_fps * plan.time_step_seconds);
        const double capped_stride = static_cast<double>(horizon) / (options.animation_max_frames - 1);
        const int stride = static_cast<int>(std::min(static_cast<double>(std::max(1, horizon)),
                                                    std::max(1.0, std::ceil(std::max(desired_stride, capped_stride)))));
        std::set<int> ticks{0, horizon};
        for (int tick = stride; tick < horizon;)
        {
            ticks.insert(tick);
            if (horizon - tick <= stride)
                break;
            tick += stride;
        }
        // Include short coordination waits if the frame budget allows it.
        for (const auto &events : plan.traffic_events)
            for (const auto &event : events)
                if (event.type == "wait" && event.end_tick > event.start_tick &&
                    event.start_tick >= 0 && event.start_tick <= horizon &&
                    ticks.size() < static_cast<std::size_t>(options.animation_max_frames))
                    ticks.insert(event.start_tick);
        const bool capped = capped_stride > std::max(1.0, std::ceil(desired_stride));
        const double speedup = capped ? stride * options.animation_fps * plan.time_step_seconds :
                                       options.animation_speedup;
        GifWriter writer(path, base.cols, base.rows);
        cv::Mat trails = base.clone();
        std::vector<std::size_t> previous(robots.size(), 0U);
        const std::vector<int> timeline(ticks.begin(), ticks.end());
        for (std::size_t f = 0; f < timeline.size(); ++f)
        {
            const int tick = timeline[f];
            for (std::size_t r = 0; r < robots.size(); ++r)
            {
                const auto &schedule = plan.schedules[r];
                const auto current = state_at(schedule, tick);
                for (std::size_t i = previous[r] + 1; i <= current; ++i)
                    if (schedule[i].position.map_id == schedule[i - 1].position.map_id &&
                        schedule[i].position != schedule[i - 1].position)
                        cv::line(trails, point(schedule[i - 1].position), point(schedule[i].position),
                                 colors[r % colors.size()], 2, cv::LINE_AA);
                previous[r] = current;
            }
            auto frame = trails.clone();
            label(frame, "t = " + seconds(tick * plan.time_step_seconds) + " / " +
                  seconds(horizon * plan.time_step_seconds) + " s   Playback ~" + seconds(speedup) + "x",
                  {15, 50}, ink, 0.55);
            for (std::size_t r = 0; r < robots.size(); ++r)
            {
                const auto &schedule = plan.schedules[r];
                const auto &state = schedule[previous[r]];
                const auto color = colors[r % colors.size()];
                const auto p = point(state.position);
                bool waiting = false;
                int wait_ticks = 0;
                const TrafficEvent *last_wait = nullptr;
                if (r < plan.traffic_events.size())
                    for (const auto &event : plan.traffic_events[r])
                        if (event.type == "wait" && tick >= event.start_tick)
                        {
                            wait_ticks += std::max(0, std::min(tick, event.end_tick) - event.start_tick);
                            waiting = waiting || tick < event.end_tick;
                            last_wait = &event;
                        }
                std::string status = tick >= schedule.back().tick ? "DONE" : "MOVE";
                bool service = false;
                if (r < plan.navigation_checkpoints.size())
                    for (const auto &checkpoint : plan.navigation_checkpoints[r])
                        if (checkpoint.type == NavigationCheckpointType::Task &&
                            tick >= checkpoint.arrival_tick && tick < checkpoint.departure_tick)
                            service = true;
                if (!state.transition_id.empty() && status != "DONE")
                    status = "TRANSIT";
                if (service && status != "DONE")
                    status = "SERVICE";
                if (waiting)
                    status = "WAIT (coordination)";
                if (waiting)
                    cv::circle(frame, p, 13, amber, 3, cv::LINE_AA);
                if (service && !waiting)
                    cv::rectangle(frame, p - cv::Point(11, 11), p + cv::Point(11, 11), color, 2);
                cv::circle(frame, p, 7, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
                cv::circle(frame, p, 6, color, -1, cv::LINE_AA);
                label(frame, std::to_string(r + 1), p + cv::Point(9, 14), color);
                const int y = 162 + static_cast<int>(r) * 90;
                label(frame, std::to_string(r + 1) + ". " + robots[r].id, {map_width + 12, y}, color);
                label(frame, status + " | " + state.position.map_id, {map_width + 12, y + 20}, waiting ? amber : ink);
                label(frame, "Wait elapsed: " + seconds(wait_ticks * plan.time_step_seconds) + " s", {map_width + 12, y + 40});
                if (last_wait)
                    label(frame, "Last wait @" + seconds(last_wait->start_tick * plan.time_step_seconds) + " s: " +
                          seconds((last_wait->end_tick - last_wait->start_tick) * plan.time_step_seconds) + " s",
                          {map_width + 12, y + 58}, ink, 0.38);
            }
            const int delay = f + 1U == timeline.size() ? 100 :
                static_cast<int>(std::clamp(std::round((timeline[f + 1] - tick) * plan.time_step_seconds /
                                                       speedup * 100), 2.0, 65535.0));
            writer.frame(frame, delay);
        }
        writer.close();
#else
        (void)path;
        (void)bundle;
        (void)tasks;
#endif
    }
}
