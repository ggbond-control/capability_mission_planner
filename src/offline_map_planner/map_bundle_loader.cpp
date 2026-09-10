#include <capability_mission_planner/offline_map_planner.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace capability_mission_planner::offline
{
    namespace
    {
        constexpr double pi = 3.14159265358979323846;
        constexpr std::uint64_t map_cache_magic = 0x434d504d415000ULL; // "CMPMAP"
        constexpr std::uint32_t max_cache_string_size = 4096U;

        struct SourceFingerprint
        {
            std::uint64_t size = 0U;
            std::int64_t modified_ticks = 0;
        };

        bool fingerprint(const std::filesystem::path &path, SourceFingerprint &output)
        {
            std::error_code error;
            const auto size = std::filesystem::file_size(path, error);
            if (error)
                return false;
            const auto modified = std::filesystem::last_write_time(path, error);
            if (error)
                return false;
            output.size = size;
            output.modified_ticks = static_cast<std::int64_t>(
                modified.time_since_epoch().count());
            return true;
        }

        std::uint64_t fnv1a_text(const std::string &value)
        {
            std::uint64_t hash = 1469598103934665603ULL;
            for (const auto character : value)
            {
                hash ^= static_cast<unsigned char>(character);
                hash *= 1099511628211ULL;
            }
            return hash;
        }

        std::filesystem::path cache_path_for(
            const std::filesystem::path &yaml_path, const std::string &map_id,
            const MapLoadOptions &options)
        {
            const auto directory = options.cache_directory.empty()
                                       ? yaml_path.parent_path() / ".capability_mission_cache"
                                       : options.cache_directory;
            std::ostringstream name;
            name << map_id << '-' << std::hex << fnv1a_text(yaml_path.string()) << ".bin";
            return directory / name.str();
        }

        template <typename T>
        bool read_binary(std::istream &input, T &value)
        {
            input.read(reinterpret_cast<char *>(&value), sizeof(value));
            return static_cast<bool>(input);
        }

        template <typename T>
        bool write_binary(std::ostream &output, const T &value)
        {
            output.write(reinterpret_cast<const char *>(&value), sizeof(value));
            return static_cast<bool>(output);
        }

        bool read_string(std::istream &input, std::string &value)
        {
            std::uint32_t size = 0U;
            if (!read_binary(input, size) || size > max_cache_string_size)
                return false;
            value.resize(size);
            input.read(value.data(), static_cast<std::streamsize>(size));
            return static_cast<bool>(input);
        }

        bool write_string(std::ostream &output, const std::string &value)
        {
            if (value.size() > max_cache_string_size)
                return false;
            const auto size = static_cast<std::uint32_t>(value.size());
            return write_binary(output, size) &&
                   (size == 0U || static_cast<bool>(output.write(value.data(), size)));
        }

        bool matches_options(const MapLoadOptions &options, bool allow_unknown,
                             double inflation_radius, double inscribed_radius, double cost_scaling_factor)
        {
            return options.allow_unknown == allow_unknown &&
                   options.inflation_radius == inflation_radius &&
                   options.inscribed_radius == inscribed_radius &&
                   options.cost_scaling_factor == cost_scaling_factor;
        }

        bool load_cached_map(
            MapLayer &map, const std::filesystem::path &path, const MapLoadOptions &options,
            double &validation_seconds, double &read_seconds)
        {
            const auto validation_started = std::chrono::steady_clock::now();
            std::ifstream input(path, std::ios::binary);
            std::uint64_t magic = 0;
            SourceFingerprint yaml_source;
            SourceFingerprint image_source;
            bool allow_unknown = false;
            double inflation_radius = 0.0;
            double inscribed_radius = 0.0;
            double cost_scaling_factor = 0.0;
            std::int32_t width = 0;
            std::int32_t height = 0;
            std::string id;
            std::string image_path;
            double resolution = 0.0;
            double origin_x = 0.0;
            double origin_y = 0.0;
            double origin_yaw = 0.0;
            if (!input || !read_binary(input, magic) || magic != map_cache_magic ||
                !read_binary(input, yaml_source.size) ||
                !read_binary(input, yaml_source.modified_ticks) ||
                !read_binary(input, image_source.size) ||
                !read_binary(input, image_source.modified_ticks) ||
                !read_binary(input, allow_unknown) ||
                !read_binary(input, inflation_radius) ||
                !read_binary(input, inscribed_radius) ||
                !read_binary(input, cost_scaling_factor) ||
                !read_string(input, id) || !read_string(input, image_path) ||
                !read_binary(input, width) || !read_binary(input, height) ||
                !read_binary(input, resolution) || !read_binary(input, origin_x) ||
                !read_binary(input, origin_y) || !read_binary(input, origin_yaw) ||
                id != map.id || width <= 0 || height <= 0 || !(resolution > 0.0) ||
                !matches_options(options, allow_unknown, inflation_radius, inscribed_radius,
                                 cost_scaling_factor))
                return false;
            SourceFingerprint current_yaml;
            const auto cached_image_path = std::filesystem::path(image_path);
            SourceFingerprint current_image;
            if (!fingerprint(map.yaml_path, current_yaml) ||
                !fingerprint(cached_image_path, current_image) ||
                current_yaml.size != yaml_source.size ||
                current_yaml.modified_ticks != yaml_source.modified_ticks ||
                current_image.size != image_source.size ||
                current_image.modified_ticks != image_source.modified_ticks)
            {
                return false;
            }
            const auto count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
            if (count == 0U || count > 100000000U)
                return false;
            validation_seconds = std::chrono::duration<double>(
                                     std::chrono::steady_clock::now() - validation_started)
                                     .count();
            const auto read_started = std::chrono::steady_clock::now();
            map.traversable.resize(count);
            map.clearance_m.resize(count);
            map.inflated_cost.resize(count);
            input.read(reinterpret_cast<char *>(map.traversable.data()),
                       static_cast<std::streamsize>(map.traversable.size()));
            input.read(reinterpret_cast<char *>(map.clearance_m.data()),
                       static_cast<std::streamsize>(map.clearance_m.size() * sizeof(float)));
            input.read(reinterpret_cast<char *>(map.inflated_cost.data()),
                       static_cast<std::streamsize>(map.inflated_cost.size()));
            if (!input)
                return false;
            read_seconds = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - read_started)
                               .count();
            map.image_path = cached_image_path;
            map.width = width;
            map.height = height;
            map.resolution = resolution;
            map.origin_x = origin_x;
            map.origin_y = origin_y;
            map.origin_yaw = origin_yaw;
            return true;
        }

        void save_cached_map(const MapLayer &map, const std::filesystem::path &path,
                             const MapLoadOptions &options)
        {
            SourceFingerprint yaml_source;
            SourceFingerprint image_source;
            if (!fingerprint(map.yaml_path, yaml_source) || !fingerprint(map.image_path, image_source))
                return;
            std::error_code error;
            std::filesystem::create_directories(path.parent_path(), error);
            if (error)
                return;
            const auto temporary = path.string() + ".tmp";
            std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
            const std::int32_t width = map.width;
            const std::int32_t height = map.height;
            if (!write_binary(output, map_cache_magic) ||
                !write_binary(output, yaml_source.size) ||
                !write_binary(output, yaml_source.modified_ticks) ||
                !write_binary(output, image_source.size) ||
                !write_binary(output, image_source.modified_ticks) ||
                !write_binary(output, options.allow_unknown) ||
                !write_binary(output, options.inflation_radius) ||
                !write_binary(output, options.inscribed_radius) ||
                !write_binary(output, options.cost_scaling_factor) ||
                !write_string(output, map.id) || !write_string(output, map.image_path.string()) ||
                !write_binary(output, width) || !write_binary(output, height) ||
                !write_binary(output, map.resolution) || !write_binary(output, map.origin_x) ||
                !write_binary(output, map.origin_y) || !write_binary(output, map.origin_yaw))
            {
                return;
            }
            output.write(reinterpret_cast<const char *>(map.traversable.data()),
                         static_cast<std::streamsize>(map.traversable.size()));
            output.write(reinterpret_cast<const char *>(map.clearance_m.data()),
                         static_cast<std::streamsize>(map.clearance_m.size() * sizeof(float)));
            output.write(reinterpret_cast<const char *>(map.inflated_cost.data()),
                         static_cast<std::streamsize>(map.inflated_cost.size()));
            output.close();
            if (!output)
                return;
            std::filesystem::rename(temporary, path, error);
            if (error)
            {
                std::filesystem::remove(path, error);
                error.clear();
                std::filesystem::rename(temporary, path, error);
            }
        }

        std::string trim(std::string value)
        {
            const auto first = value.find_first_not_of(" \t\r\n");
            if (first == std::string::npos)
                return {};
            const auto last = value.find_last_not_of(" \t\r\n");
            return value.substr(first, last - first + 1U);
        }

        std::vector<std::string> csv_fields(const std::string &line)
        {
            std::vector<std::string> result;
            std::string field;
            bool quoted = false;
            for (std::size_t i = 0; i < line.size(); ++i)
            {
                const char character = line[i];
                if (character == '"')
                {
                    if (quoted && i + 1U < line.size() && line[i + 1U] == '"')
                    {
                        field.push_back('"');
                        ++i;
                    }
                    else
                    {
                        quoted = !quoted;
                    }
                }
                else if (character == ',' && !quoted)
                {
                    result.push_back(trim(field));
                    field.clear();
                }
                else
                {
                    field.push_back(character);
                }
            }
            if (quoted)
                throw std::runtime_error("unterminated quoted CSV field");
            result.push_back(trim(field));
            return result;
        }

        bool parse_bool(const std::string &value)
        {
            if (value == "true" || value == "TRUE" || value == "1")
                return true;
            if (value == "false" || value == "FALSE" || value == "0")
                return false;
            throw std::runtime_error("invalid boolean value: " + value);
        }

        double occupancy_from_pixel(unsigned char pixel, bool negate)
        {
            const double normalized = static_cast<double>(pixel) / 255.0;
            return negate ? normalized : 1.0 - normalized;
        }

        MapLayer load_map(const std::filesystem::path &yaml_path, const MapLoadOptions &options)
        {
            if (options.inflation_radius < 0.0 || options.inscribed_radius < 0.0 ||
                options.cost_scaling_factor <= 0.0)
            {
                throw std::runtime_error(yaml_path.string() + ": invalid clearance cost parameters");
            }
            MapLayer map;
            map.id = yaml_path.stem().string();
            map.yaml_path = std::filesystem::absolute(yaml_path);
            const auto cache_path = options.persistent_cache ? cache_path_for(map.yaml_path, map.id, options) : std::filesystem::path{};
            if (!cache_path.empty() && load_cached_map(map, cache_path, options,
                                                       map.cache_validation_seconds, map.cache_read_seconds))
            {
                map.loaded_from_persistent_cache = true;
                return map;
            }

            const auto preprocess_started = std::chrono::steady_clock::now();
            const auto yaml = YAML::LoadFile(yaml_path.string());
            for (const auto *key : {"image", "resolution", "origin"})
            {
                if (!yaml[key])
                    throw std::runtime_error(yaml_path.string() + ": missing " + key);
            }

            map.image_path = yaml["image"].as<std::string>();
            if (map.image_path.is_relative())
                map.image_path = map.yaml_path.parent_path() / map.image_path;
            map.image_path = std::filesystem::absolute(map.image_path);
            map.resolution = yaml["resolution"].as<double>();
            if (!(map.resolution > 0.0))
                throw std::runtime_error(yaml_path.string() + ": resolution must be positive");
            const auto origin = yaml["origin"];
            if (!origin.IsSequence() || origin.size() < 3U)
                throw std::runtime_error(yaml_path.string() + ": origin must contain x, y and yaw");
            map.origin_x = origin[0].as<double>();
            map.origin_y = origin[1].as<double>();
            map.origin_yaw = origin[2].as<double>();

            const bool negate = yaml["negate"] ? yaml["negate"].as<int>() != 0 : false;
            const double occupied_threshold =
                yaml["occupied_thresh"] ? yaml["occupied_thresh"].as<double>() : 0.65;
            const double free_threshold =
                yaml["free_thresh"] ? yaml["free_thresh"].as<double>() : 0.196;
            const std::string mode = yaml["mode"] ? yaml["mode"].as<std::string>() : "trinary";
            if (occupied_threshold < 0.0 || occupied_threshold > 1.0 ||
                free_threshold < 0.0 || free_threshold > 1.0 ||
                free_threshold > occupied_threshold)
            {
                throw std::runtime_error(yaml_path.string() + ": invalid occupancy thresholds");
            }
            if (mode != "trinary" && mode != "scale" && mode != "raw")
                throw std::runtime_error(yaml_path.string() + ": unsupported map mode " + mode);
            const auto image = cv::imread(map.image_path.string(), cv::IMREAD_GRAYSCALE);
            if (image.empty())
                throw std::runtime_error("cannot read map image: " + map.image_path.string());
            map.width = image.cols;
            map.height = image.rows;
            cv::Mat free_mask(image.size(), CV_8UC1, cv::Scalar(0));
            for (int row = 0; row < image.rows; ++row)
            {
                for (int column = 0; column < image.cols; ++column)
                {
                    const auto pixel = image.at<unsigned char>(row, column);
                    bool free = false;
                    if (mode == "raw")
                    {
                        free = pixel <= 100U && pixel != 255U &&
                               static_cast<double>(pixel) / 100.0 <= free_threshold;
                    }
                    else
                    {
                        const double occupancy = occupancy_from_pixel(pixel, negate);
                        if (occupancy <= free_threshold)
                            free = true;
                        else if (occupancy < occupied_threshold)
                            free = options.allow_unknown;
                    }
                    free_mask.at<unsigned char>(row, column) = free ? 255U : 0U;
                }
            }

            cv::Mat distance_pixels;
            cv::distanceTransform(free_mask, distance_pixels, cv::DIST_L2, 3);
            map.clearance_m.resize(static_cast<std::size_t>(map.width * map.height));
            map.inflated_cost.resize(static_cast<std::size_t>(map.width * map.height));
            for (int y = 0; y < map.height; ++y)
            {
                const int row = map.height - 1 - y;
                for (int x = 0; x < map.width; ++x)
                {
                    const double clearance = static_cast<double>(distance_pixels.at<float>(row, x)) *
                                             map.resolution;
                    const auto index = static_cast<std::size_t>(y * map.width + x);
                    map.clearance_m[index] = static_cast<float>(clearance);
                    if (free_mask.at<unsigned char>(row, x) == 0U)
                    {
                        map.inflated_cost[index] = 254U;
                    }
                    else if (clearance <= options.inscribed_radius)
                    {
                        map.inflated_cost[index] = 253U;
                    }
                    else
                    {
                        const double factor = std::exp(-options.cost_scaling_factor *
                                                       (clearance - options.inscribed_radius));
                        map.inflated_cost[index] = static_cast<unsigned char>(std::clamp(
                            252.0 * factor, 0.0, 252.0));
                    }
                }
            }

            if (options.inflation_radius > 0.0)
            {
                const int radius = static_cast<int>(std::ceil(
                    options.inflation_radius / map.resolution));
                cv::Mat blocked;
                cv::bitwise_not(free_mask, blocked);
                const auto kernel = cv::getStructuringElement(
                    cv::MORPH_ELLIPSE, {2 * radius + 1, 2 * radius + 1});
                cv::dilate(blocked, blocked, kernel);
                cv::bitwise_not(blocked, free_mask);
            }

            map.traversable.resize(static_cast<std::size_t>(map.width * map.height));
            for (int y = 0; y < map.height; ++y)
            {
                const int row = map.height - 1 - y;
                for (int x = 0; x < map.width; ++x)
                {
                    map.traversable[static_cast<std::size_t>(y * map.width + x)] =
                        free_mask.at<unsigned char>(row, x) != 0U;
                }
            }
            if (!cache_path.empty())
                save_cached_map(map, cache_path, options);
            map.preprocess_seconds = std::chrono::duration<double>(
                                         std::chrono::steady_clock::now() - preprocess_started)
                                         .count();
            return map;
        }

        void load_relations(MultiMapBundle &bundle, const std::filesystem::path &path)
        {
            if (!std::filesystem::exists(path))
            {
                if (bundle.maps.size() == 1U)
                    return;
                throw std::runtime_error("multi-map directory is missing map_relations.csv");
            }
            std::ifstream input(path);
            std::string line;
            if (!std::getline(input, line) ||
                csv_fields(line) != std::vector<std::string>{"from_map", "to_map", "dx", "dy"})
            {
                throw std::runtime_error(path.string() + ": invalid header");
            }

            using Edge = std::tuple<std::string, double, double>;
            std::map<std::string, std::vector<Edge>> graph;
            while (std::getline(input, line))
            {
                if (trim(line).empty())
                    continue;
                const auto fields = csv_fields(line);
                if (fields.size() != 4U)
                    throw std::runtime_error(path.string() + ": relation row must have 4 fields");
                const double dx = std::stod(fields[2]);
                const double dy = std::stod(fields[3]);
                if (fields[0] != "ROOT" && bundle.maps.count(fields[0]) == 0U)
                    throw std::runtime_error(path.string() + ": unknown map " + fields[0]);
                if (fields[1] != "ROOT" && bundle.maps.count(fields[1]) == 0U)
                    throw std::runtime_error(path.string() + ": unknown map " + fields[1]);
                graph[fields[0]].emplace_back(fields[1], dx, dy);
                graph[fields[1]].emplace_back(fields[0], -dx, -dy);
            }

            std::map<std::string, std::pair<double, double>> offsets{{"ROOT", {0.0, 0.0}}};
            std::deque<std::string> queue{"ROOT"};
            while (!queue.empty())
            {
                const auto current = queue.front();
                queue.pop_front();
                for (const auto &[next, dx, dy] : graph[current])
                {
                    const auto candidate = std::make_pair(
                        offsets[current].first + dx, offsets[current].second + dy);
                    const auto existing = offsets.find(next);
                    if (existing == offsets.end())
                    {
                        offsets[next] = candidate;
                        queue.push_back(next);
                    }
                    else if (std::abs(existing->second.first - candidate.first) > 1e-6 ||
                             std::abs(existing->second.second - candidate.second) > 1e-6)
                    {
                        throw std::runtime_error(path.string() + ": inconsistent relation graph at " + next);
                    }
                }
            }
            for (auto &[id, map] : bundle.maps)
            {
                const auto offset = offsets.find(id);
                if (offset == offsets.end())
                    throw std::runtime_error(path.string() + ": no ROOT path to " + id);
                map.root_x = offset->second.first;
                map.root_y = offset->second.second;
            }
        }

        void load_transitions(MultiMapBundle &bundle, const std::filesystem::path &path)
        {
            if (!std::filesystem::exists(path))
            {
                if (bundle.maps.size() > 1U)
                    throw std::runtime_error("multi-map directory is missing transition_points.csv");
                return;
            }
            std::ifstream input(path);
            std::string line;
            const std::vector<std::string> expected{
                "transition_id", "from_map", "to_map", "world_x", "world_y", "world_z",
                "world_yaw_rad", "bidirectional", "type"};
            if (!std::getline(input, line) || csv_fields(line) != expected)
                throw std::runtime_error(path.string() + ": invalid header");

            std::set<std::tuple<std::string, std::string, std::string>> directed_transitions;
            const auto add = [&](MapTransition transition)
            {
                if (!directed_transitions.insert(
                                             {transition.id, transition.from_map, transition.to_map})
                         .second)
                {
                    throw std::runtime_error(path.string() + ": duplicate transition " +
                                             transition.id + " on " + transition.from_map + " -> " + transition.to_map);
                }
                const auto &from = bundle.map(transition.from_map);
                const auto &to = bundle.map(transition.to_map);
                transition.from_cell = from.local_to_grid(
                    from.root_to_local(transition.root_pose).x,
                    from.root_to_local(transition.root_pose).y);
                transition.to_cell = to.local_to_grid(
                    to.root_to_local(transition.root_pose).x,
                    to.root_to_local(transition.root_pose).y);
                if (!bundle.traversable(transition.from_cell) ||
                    !bundle.traversable(transition.to_cell))
                {
                    throw std::runtime_error(path.string() + ": transition " + transition.id +
                                             " is not traversable on both maps");
                }
                bundle.transitions.push_back(std::move(transition));
            };

            while (std::getline(input, line))
            {
                if (trim(line).empty())
                    continue;
                const auto fields = csv_fields(line);
                if (fields.size() != 9U)
                    throw std::runtime_error(path.string() + ": transition row must have 9 fields");
                if (bundle.maps.count(fields[1]) == 0U || bundle.maps.count(fields[2]) == 0U)
                    throw std::runtime_error(path.string() + ": transition references unknown map");
                MapTransition transition;
                transition.id = fields[0];
                transition.from_map = fields[1];
                transition.to_map = fields[2];
                transition.root_pose = MetricPose{
                    "ROOT", std::stod(fields[3]), std::stod(fields[4]), std::stod(fields[5]),
                    std::stod(fields[6])};
                transition.bidirectional = parse_bool(fields[7]);
                transition.type = fields[8];
                add(transition);
                if (transition.bidirectional)
                {
                    std::swap(transition.from_map, transition.to_map);
                    add(std::move(transition));
                }
            }
        }

    } // namespace

    bool GridPosition::operator==(const GridPosition &other) const
    {
        return map_id == other.map_id && x == other.x && y == other.y;
    }

    bool GridPosition::operator<(const GridPosition &other) const
    {
        return std::tie(map_id, x, y) < std::tie(other.map_id, other.x, other.y);
    }

    bool MapLayer::is_traversable(int x, int y) const
    {
        return x >= 0 && x < width && y >= 0 && y < height &&
               traversable[static_cast<std::size_t>(y * width + x)] != 0U;
    }

    double MapLayer::clearance(int x, int y) const
    {
        if (x < 0 || x >= width || y < 0 || y >= height || clearance_m.empty())
            return 0.0;
        return clearance_m[static_cast<std::size_t>(y * width + x)];
    }

    unsigned char MapLayer::cost(int x, int y) const
    {
        if (x < 0 || x >= width || y < 0 || y >= height || inflated_cost.empty())
            return 254U;
        return inflated_cost[static_cast<std::size_t>(y * width + x)];
    }

    GridPosition MapLayer::local_to_grid(double x, double y) const
    {
        const double dx = x - origin_x;
        const double dy = y - origin_y;
        const double cosine = std::cos(origin_yaw);
        const double sine = std::sin(origin_yaw);
        const double local_x = cosine * dx + sine * dy;
        const double local_y = -sine * dx + cosine * dy;
        return {id, static_cast<int>(std::floor(local_x / resolution + 1e-9)),
                static_cast<int>(std::floor(local_y / resolution + 1e-9))};
    }

    MetricPose MapLayer::grid_to_local(const GridPosition &position) const
    {
        if (position.map_id != id)
            throw std::invalid_argument("grid position belongs to a different map");
        const double gx = (static_cast<double>(position.x) + 0.5) * resolution;
        const double gy = (static_cast<double>(position.y) + 0.5) * resolution;
        const double cosine = std::cos(origin_yaw);
        const double sine = std::sin(origin_yaw);
        return {id, origin_x + cosine * gx - sine * gy,
                origin_y + sine * gx + cosine * gy, 0.0, origin_yaw};
    }

    MetricPose MapLayer::local_to_root(const MetricPose &pose) const
    {
        const double cosine = std::cos(root_yaw);
        const double sine = std::sin(root_yaw);
        return {"ROOT", root_x + cosine * pose.x - sine * pose.y,
                root_y + sine * pose.x + cosine * pose.y, pose.z,
                std::remainder(root_yaw + pose.yaw, 2.0 * pi)};
    }

    MetricPose MapLayer::root_to_local(const MetricPose &pose) const
    {
        const double dx = pose.x - root_x;
        const double dy = pose.y - root_y;
        const double cosine = std::cos(root_yaw);
        const double sine = std::sin(root_yaw);
        return {id, cosine * dx + sine * dy, -sine * dx + cosine * dy, pose.z,
                std::remainder(pose.yaw - root_yaw, 2.0 * pi)};
    }

    const MapLayer &MultiMapBundle::map(const std::string &id) const
    {
        const auto found = maps.find(id);
        if (found == maps.end())
            throw std::out_of_range("unknown map: " + id);
        return found->second;
    }

    bool MultiMapBundle::traversable(const GridPosition &position) const
    {
        const auto found = maps.find(position.map_id);
        return found != maps.end() && found->second.is_traversable(position.x, position.y);
    }

    std::shared_ptr<const MultiMapBundle> MapBundleLoader::load(
        const std::filesystem::path &directory,
        const MapLoadOptions &options)
    {
        if (!std::filesystem::is_directory(directory))
            throw std::invalid_argument("map bundle directory does not exist: " + directory.string());
        auto bundle = std::make_shared<MultiMapBundle>();
        bundle->directory = std::filesystem::absolute(directory);
        for (const auto &entry : std::filesystem::directory_iterator(directory))
        {
            if (!entry.is_regular_file() || entry.path().extension() != ".yaml")
                continue;
            auto map = load_map(entry.path(), options);
            if (map.loaded_from_persistent_cache)
                ++bundle->map_cache_hits;
            else
                ++bundle->map_cache_misses;
            bundle->map_cache_validation_seconds += map.cache_validation_seconds;
            bundle->map_cache_read_seconds += map.cache_read_seconds;
            bundle->map_preprocess_seconds += map.preprocess_seconds;
            if (!bundle->maps.emplace(map.id, std::move(map)).second)
                throw std::runtime_error("duplicate map id in " + directory.string());
        }
        if (bundle->maps.empty())
            throw std::runtime_error("map bundle contains no YAML maps: " + directory.string());
        load_relations(*bundle, directory / "map_relations.csv");
        load_transitions(*bundle, directory / "transition_points.csv");
        return bundle;
    }
} // namespace capability_mission_planner::offline
