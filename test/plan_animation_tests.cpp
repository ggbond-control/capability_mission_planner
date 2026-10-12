#include <capability_mission_planner/offline_planner_config.hpp>
#include "../src/offline_map_planner/multi_map_coordinator.hpp"

#include <gif_lib.h>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

using namespace capability_mission_planner::offline;

namespace
{
    void require(bool condition, const char *message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    cv::Vec3b gif_pixel(const GifFileType &gif, int frame, int x, int y)
    {
        // Composite transparent delta frames according to the GIF control block.
        for (int f = frame; f >= 0; --f)
        {
            const auto &image = gif.SavedImages[f];
            GraphicsControlBlock control{};
            DGifSavedExtensionToGCB(const_cast<GifFileType *>(&gif), f, &control);
            const auto value = image.RasterBits[y * image.ImageDesc.Width + x];
            if (value == control.TransparentColor)
                continue;
            const auto *palette = image.ImageDesc.ColorMap ? image.ImageDesc.ColorMap : gif.SColorMap;
            const auto color = palette->Colors[value];
            return {color.Blue, color.Green, color.Red};
        }
        throw std::runtime_error("GIF pixel has no opaque background");
    }

    void check_gif(const std::filesystem::path &path, int maximum_frames,
                   const std::vector<MappedRobot> &robots, const OfflineMissionPlan &plan,
                   bool multi_map)
    {
        int error = 0;
        auto *gif = DGifOpenFileName(path.string().c_str(), &error);
        require(gif != nullptr, "cannot open exported GIF");
        require(DGifSlurp(gif) == GIF_OK, "exported GIF cannot be decoded");
        require(gif->ImageCount >= 2 && gif->ImageCount <= maximum_frames, "GIF frame limit violated");
        require(gif->SWidth == 1280 && gif->SHeight > 240, "GIF canvas dimensions incorrect");
        for (int f = 0; f < gif->ImageCount; ++f)
        {
            GraphicsControlBlock control{};
            require(DGifSavedExtensionToGCB(gif, f, &control) == GIF_OK && control.DelayTime > 0,
                    "GIF frame delay missing");
        }
        GraphicsControlBlock last{};
        DGifSavedExtensionToGCB(gif, gif->ImageCount - 1, &last);
        require(last.DelayTime == 100, "final frame must hold for one second");
        if (!multi_map)
        {
            // Check actual robot pixels, independently of the renderer's code.
            // 80x60 fixture fits in a 980x765 tile, limited by the 960px usable width.
            const double scale = 960.0 / 80;
            const int left = (980 - static_cast<int>(80 * scale)) / 2;
            const auto center = [&](const GridPosition &p)
            {
                return cv::Point(left + std::lround((p.x + 0.5) * scale),
                                 125 + std::lround((60 - p.y - 0.5) * scale));
            };
            const auto start = center(robots[0].start);
            const auto finish = center(plan.schedules[0].back().position);
            const cv::Vec3b red{51, 51, 204};
            require(gif_pixel(*gif, 0, start.x, start.y) == red, "first GIF frame has wrong robot position");
            require(gif_pixel(*gif, gif->ImageCount - 1, finish.x, finish.y) == red,
                    "final GIF frame has wrong robot position");
            bool amber_visible = false;
            for (int f = 0; f < gif->ImageCount; ++f)
                for (int y = start.y - 17; y <= start.y + 17; ++y)
                    for (int x = start.x - 17; x <= start.x + 17; ++x)
                        amber_visible = amber_visible || gif_pixel(*gif, f, x, y) == cv::Vec3b(0, 153, 255);
            require(amber_visible, "coordination wait halo missing from GIF");
        }
        require(DGifCloseFile(gif, &error) == GIF_OK, "cannot close decoded GIF");
    }
}

int main(int argc, char *argv[])
{
    const auto directory = std::filesystem::temp_directory_path() /
                           ("cmp_animation_test_" + std::to_string(getpid()));
    try
    {
        std::filesystem::create_directories(directory);
        auto bundle = std::make_shared<MultiMapBundle>();
        MapLayer map;
        map.id = "fixture";
        map.width = 80;
        map.height = 60;
        map.resolution = 1.0;
        map.traversable.assign(80 * 60, 1);
        map.clearance_m.assign(80 * 60, 100.0F);
        map.inflated_cost.assign(80 * 60, 0);
        map.image_path = directory / "map.png";
        cv::imwrite(map.image_path.string(), cv::Mat(60, 80, CV_8UC3, cv::Scalar(255, 255, 255)));
        bundle->maps.emplace(map.id, map);
        std::vector<MappedRobot> robots{
            {"first", {map.id, 20, 30}, {}, {}}, {"second", {map.id, 30, 30}, {}, {}}};
        OfflineMissionPlan plan;
        plan.routes.resize(2);
        plan.routes[0].robot_index = 0;
        plan.routes[0].segments = {{{{{map.id, 20, 30}, 0, {}}, {{map.id, 30, 30}, 1, {}},
                                    {{map.id, 40, 30}, 2, {}}}, 2}};
        plan.routes[1].robot_index = 1;
        // The second robot must service its starting cell before leaving it.
        // Prioritized planning fixes the first robot too early; CBS delays it.
        plan.routes[1].stops = {{{map.id, 30, 30}, {}, 3, 0.0}};
        plan.routes[1].segments = {
            {{{{map.id, 30, 30}, 0, {}}}, 0},
            {{{{map.id, 30, 30}, 0, {}}, {{map.id, 30, 40}, 1, {}}, {{map.id, 30, 50}, 2, {}}}, 2}};
        plan.routes[0].travel_ticks = 2;
        plan.routes[1].travel_ticks = 2;
        plan.routes[1].service_ticks = 3;
        plan.maximum_load_ticks = 5;
        plan.total_load_ticks = 7;
        TraversalOptions traversal;
        traversal.time_step_seconds = 1.0;
        plan.time_step_seconds = 1.0;
        plan.schedules = coordinate_multi_map_routes(MultiMapPathPlanner(bundle, traversal), robots,
                                                     plan.routes, &plan.coordination_stats);
        require(!plan.coordination_stats.prioritized_succeeded && plan.coordination_stats.cbs_succeeded,
                "crossing fixture must exercise CBS fallback");
        require(plan.coordination_stats.total_wait_ticks > 0, "CBS fixture must insert coordination waits");
        plan.navigation_checkpoints.resize(2);
        plan.navigation_checkpoints[1].push_back({NavigationCheckpointType::Task, robots[1].start,
                                                 0, 3, {}, {}, "service-3s", "service"});
        plan.traffic_events.resize(2);
        for (std::size_t r = 0; r < plan.schedules.size(); ++r)
        {
            const auto &schedule = plan.schedules[r];
            for (std::size_t i = 0; i + 1 < schedule.size(); ++i)
            {
                std::size_t end = i;
                while (end + 1 < schedule.size() && schedule[end + 1].route_frame == schedule[i].route_frame)
                    ++end;
                if (end > i)
                    plan.traffic_events[r].push_back({"wait", schedule[i].tick, schedule[end].tick,
                                                     schedule[i].position, {}, "coordination_delay", "wait"});
                i = end;
            }
        }
        ExportOptions options;
        options.animation = true;
        options.animation_speedup = 1.0;
        options.animation_max_frames = 12;
        options.filter_navigation_checkpoint_types = true; // Hiding checkpoints must not affect replay.
        PlanExporter::write(directory / "single", *bundle, robots, {}, plan, options);
        const auto data = YAML::LoadFile((directory / "single/plan.json").string());
        require(data["coordination"]["method"].as<std::string>() == "cbs", "JSON must identify actual CBS run");
        require(data["routes"][0]["timed_schedule"].size() == plan.schedules[0].size(),
                "JSON must preserve exact time steps independently of checkpoint filtering");
        check_gif(directory / "single/routes_overall.gif", 12, robots, plan, false);

        // Multi-map transitions must switch panels, while completed robots stay visible.
        auto second = map;
        second.id = "other";
        bundle->maps.emplace(second.id, second);
        plan.schedules[1].back().position.map_id = second.id;
        plan.schedules[1].back().transition_id = "portal";
        options.animation_max_frames = 2;
        PlanExporter::write(directory / "multi", *bundle, robots, {}, plan, options);
        check_gif(directory / "multi/routes_overall.gif", 2, robots, plan, true);

        options.animation = false;
        PlanExporter::write(directory / "disabled", *bundle, robots, {}, plan, options);
        require(!std::filesystem::exists(directory / "disabled/routes_overall.gif"),
                "disabled animation must not create a GIF");
        options.animation = true;
        auto no_schedule = plan;
        no_schedule.schedules.clear();
        bool rejected = false;
        try { PlanExporter::write(directory / "invalid", *bundle, robots, {}, no_schedule, options); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected && !std::filesystem::exists(directory / "invalid"),
                "missing schedule must fail before any files are exported");
        std::filesystem::create_directories(directory / "maps");
        std::ofstream map_yaml(directory / "maps/map.yaml");
        map_yaml << "image: ../map.png\nresolution: 1.0\norigin: [0, 0, 0]\nnegate: 0\n"
                    "occupied_thresh: 0.65\nfree_thresh: 0.196\n";
        map_yaml.close();
        const auto config_path = directory / "config.yaml";
        YAML::Node config = YAML::Load(
            "map: {directory: maps}\nexport: {animation: true, animation_fps: 12, animation_speedup: 3, "
            "animation_max_frames: 8, animation_width: 960}\n"
            "robots: [{id: r, start: {map_id: map, grid: [20, 30]}, capabilities: []}]\ntasks: []\n");
        { std::ofstream file(config_path); file << config; }
        const auto loaded = OfflinePlannerConfigLoader::load(config_path);
        require(loaded.export_options.animation && loaded.export_options.animation_fps == 12 &&
                    loaded.export_options.animation_speedup == 3 && loaded.export_options.animation_max_frames == 8 &&
                    loaded.export_options.animation_width == 960, "YAML animation options were not loaded");
        config["export"]["animation_speedup"] = 0;
        { std::ofstream file(config_path); file << config; }
        rejected = false;
        try { (void)OfflinePlannerConfigLoader::load(config_path); }
        catch (const std::runtime_error &) { rejected = true; }
        require(rejected, "invalid YAML animation speedup must fail");
        if (argc == 2)
        {
            // Optional review artifact from the exact CBS fixture validated above.
            std::filesystem::create_directories(argv[1]);
            for (const auto &entry : std::filesystem::directory_iterator(directory / "single"))
                std::filesystem::copy_file(entry.path(), std::filesystem::path(argv[1]) / entry.path().filename(),
                                           std::filesystem::copy_options::overwrite_existing);
            std::filesystem::copy_file(map.image_path, std::filesystem::path(argv[1]) / "map.png",
                                       std::filesystem::copy_options::overwrite_existing);
        }
        std::filesystem::remove_all(directory);
        std::cout << "animation/CBS replay tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << " (fixture retained at " << directory << ")\n";
        return 1;
    }
}
