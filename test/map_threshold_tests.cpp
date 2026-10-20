#include <capability_mission_planner/offline/offline_planner_config.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>

using namespace capability_mission_planner::offline;

namespace
{
    void require(bool condition, const std::string &message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    template <typename Function>
    void rejects(Function action)
    {
        try { action(); }
        catch (const std::exception &) { return; }
        throw std::runtime_error("invalid thresholds were accepted");
    }

    std::string contents(const std::filesystem::path &path)
    {
        std::ifstream input(path, std::ios::binary);
        return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    }

    void write_map(const std::filesystem::path &directory, const std::string &mode, bool negate = false,
                   double free = 0.0, double occupied = 1.0)
    {
        std::ofstream yaml(directory / "test.yaml");
        yaml << "image: test.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n"
             << "negate: " << negate << "\nmode: " << mode
             << "\nfree_thresh: " << free << "\noccupied_thresh: " << occupied << '\n';
    }
}

int main()
{
    const auto fixture = std::filesystem::temp_directory_path() /
        ("capability_map_thresholds_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    try
    {
        std::filesystem::create_directories(fixture);
        {
            std::ofstream image(fixture / "test.pgm", std::ios::binary);
            image << "P5\n8 2\n255\n";
            const unsigned char pixels[] = {255, 254, 253, 252, 128, 0, 50, 100};
            for (int row = 0; row < 2; ++row)
                image.write(reinterpret_cast<const char *>(pixels), sizeof(pixels));
        }
        write_map(fixture, "scale");
        const auto yaml_before = contents(fixture / "test.yaml");
        const auto image_before = contents(fixture / "test.pgm");
        MapLoadOptions options;
        options.cache_directory = fixture / "cache";
        const auto original = MapBundleLoader::load(fixture, options);
        require(original->map("test").is_traversable(0, 0) &&
                !original->map("test").is_traversable(2, 0), "inherited gray 253 must remain unknown");
        require(MapBundleLoader::load(fixture, options)->map_cache_hits == 1, "inherited cache miss");
        options.free_thresh_override = 0.01;
        const auto freed = MapBundleLoader::load(fixture, options);
        require(freed->map_cache_hits == 0 && freed->map("test").is_traversable(2, 0) &&
                !freed->map("test").is_traversable(3, 0), "free override/cache invalidation failed");
        require(MapBundleLoader::load(fixture, options)->map_cache_hits == 1, "free override cache miss");
        // occupied alone does not free unknown cells when allow_unknown=false.
        options.occupied_thresh_override = 0.5;
        const auto occupied = MapBundleLoader::load(fixture, options);
        require(occupied->map_cache_hits == 0 && occupied->map("test").traversable == freed->map("test").traversable,
                "occupied override changed free cells with allow_unknown=false");
        require(MapBundleLoader::load(fixture, options)->map_cache_hits == 1, "occupied override cache miss");
        options.allow_unknown = true;
        const auto unknown = MapBundleLoader::load(fixture, options);
        require(unknown->map("test").is_traversable(4, 0) && !unknown->map("test").is_traversable(6, 0),
                "occupied threshold must bound unknown traversal");
        options.occupied_thresh_override = 1.0;
        require(MapBundleLoader::load(fixture, options)->map("test").is_traversable(6, 0),
                "occupied override not applied");
        options.allow_unknown = false;
        options.free_thresh_override.reset();
        options.occupied_thresh_override.reset();
        require(MapBundleLoader::load(fixture, options)->map_cache_hits == 0, "reset overrides must invalidate cache");
        require(MapBundleLoader::load(fixture, options)->map("test").traversable == original->map("test").traversable,
                "reset overrides did not restore original map");
        for (double invalid : {-0.1, 1.1, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        {
            for (bool free : {false, true})
            {
                auto bad = options;
                (free ? bad.free_thresh_override : bad.occupied_thresh_override) = invalid;
                rejects([&] { MapBundleLoader::load(fixture, bad); });
            }
        }
        options.free_thresh_override = 0.6;
        options.occupied_thresh_override = 0.5;
        rejects([&] { MapBundleLoader::load(fixture, options); });
        options.free_thresh_override = 0.0;
        options.occupied_thresh_override = 0.0;
        require(MapBundleLoader::load(fixture, options)->map("test").is_traversable(0, 0), "equal zero thresholds rejected");
        options.free_thresh_override = 1.0;
        options.occupied_thresh_override = 1.0;
        require(MapBundleLoader::load(fixture, options)->map("test").is_traversable(5, 0), "equal one thresholds rejected");
        require(contents(fixture / "test.yaml") == yaml_before && contents(fixture / "test.pgm") == image_before,
                "loading changed original map files");

        // Configuration parser, including one override checked against inherited counterpart.
        auto root = YAML::Load("map: {persistent_cache: false}\nrobots: []\ntasks: []\n");
        root["map"]["directory"] = fixture.string();
        root["map"]["free_thresh_override"] = 0.01;
        root["map"]["occupied_thresh_override"] = 1.0;
        require(OfflinePlannerConfigLoader::load_node(root).bundle->map("test").is_traversable(2, 0), "config overrides not forwarded");
        for (const auto *name : {"free_thresh_override", "occupied_thresh_override"})
        {
            auto bad = YAML::Clone(root);
            bad["map"][name] = std::numeric_limits<double>::quiet_NaN();
            rejects([&] { OfflinePlannerConfigLoader::load_node(bad); });
        }
        write_map(fixture, "scale", false, 0.8, 0.9);
        auto one = YAML::Clone(root);
        one["map"].remove("free_thresh_override");
        one["map"]["occupied_thresh_override"] = 0.5;
        rejects([&] { OfflinePlannerConfigLoader::load_node(one); });
        one = YAML::Clone(root);
        one["map"].remove("occupied_thresh_override");
        one["map"]["free_thresh_override"] = 0.95;
        rejects([&] { OfflinePlannerConfigLoader::load_node(one); });

        // Preserve negate and raw mode semantics; no separate image processing path.
        options = MapLoadOptions{};
        options.persistent_cache = false;
        options.free_thresh_override = 0.01;
        options.occupied_thresh_override = 1.0;
        for (const auto *mode : {"scale", "trinary"})
        {
            write_map(fixture, mode, true);
            const auto map = MapBundleLoader::load(fixture, options);
            require(map->map("test").is_traversable(5, 0) && !map->map("test").is_traversable(0, 0),
                    "negate semantics changed");
        }
        write_map(fixture, "raw");
        options.free_thresh_override = 0.5;
        options.allow_unknown = true;
        const auto raw = MapBundleLoader::load(fixture, options);
        require(raw->map("test").is_traversable(6, 0) && !raw->map("test").is_traversable(7, 0) &&
                !raw->map("test").is_traversable(0, 0), "raw threshold/unknown semantics changed");
        std::filesystem::remove_all(fixture);
        std::cout << "map threshold tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::filesystem::remove_all(fixture);
        std::cerr << "map threshold tests failed: " << error.what() << '\n';
        return 1;
    }
}
