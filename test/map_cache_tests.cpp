#include <capability_mission_planner/offline_map_planner.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace capability_mission_planner::offline;

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

std::filesystem::path make_fixture() {
  const auto directory = std::filesystem::temp_directory_path() /
    ("capability_mission_map_cache_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(directory);
  {
    std::ofstream image(directory / "test.pgm", std::ios::binary);
    require(static_cast<bool>(image), "could not create fixture image");
    image << "P5\n8 8\n255\n";
    for (int index = 0; index < 64; ++index) {
      const char pixel = (index == 0 || index == 9) ? 0 : static_cast<char>(255);
      image.write(&pixel, 1);
    }
  }
  {
    std::ofstream yaml(directory / "test.yaml");
    require(static_cast<bool>(yaml), "could not create fixture YAML");
    yaml << "image: test.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
         << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  }
  return directory;
}

} // namespace

int main() {
  std::filesystem::path fixture;
  try {
    fixture = make_fixture();
    MapLoadOptions cached;
    cached.cache_directory = fixture / "cache";
    const auto first = MapBundleLoader::load(fixture, cached);
    require(first->map_cache_hits == 0U && first->map_cache_misses == 1U,
      "first cached load should build the cache");

    const auto second = MapBundleLoader::load(fixture, cached);
    require(second->map_cache_hits == 1U && second->map_cache_misses == 0U,
      "second cached load should use the cache");

    {
      std::ofstream yaml(fixture / "test.yaml", std::ios::app);
      require(static_cast<bool>(yaml), "could not modify fixture YAML");
      yaml << "# cache invalidation check\n";
    }
    const auto invalidated = MapBundleLoader::load(fixture, cached);
    require(invalidated->map_cache_hits == 0U && invalidated->map_cache_misses == 1U,
      "modified source YAML should invalidate the cache");

    MapLoadOptions uncached = cached;
    uncached.persistent_cache = false;
    const auto third = MapBundleLoader::load(fixture, uncached);
    require(third->map_cache_hits == 0U && third->map_cache_misses == 1U,
      "disabled cache should rebuild map fields");
    const auto& cached_map = second->map("test");
    const auto& uncached_map = third->map("test");
    require(cached_map.traversable == uncached_map.traversable &&
      cached_map.clearance_m == uncached_map.clearance_m &&
      cached_map.inflated_cost == uncached_map.inflated_cost,
      "cached and uncached map fields differ");

    std::filesystem::remove_all(fixture);
    std::cout << "map cache tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    if (!fixture.empty()) std::filesystem::remove_all(fixture);
    std::cerr << "map cache tests failed: " << error.what() << '\n';
    return 1;
  }
}
