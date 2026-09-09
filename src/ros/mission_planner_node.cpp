#include <capability_mission_planner/offline_map_planner.hpp>
#include <capability_mission_planner/offline_planner_config.hpp>

#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>

namespace capability_mission_planner::offline
{
    namespace
    {
        std::string quote_json(const std::string &value)
        {
            std::ostringstream output;
            output << '"';
            for (const unsigned char character : value)
            {
                switch (character)
                {
                case '"':
                    output << "\\\"";
                    break;
                case '\\':
                    output << "\\\\";
                    break;
                case '\n':
                    output << "\\n";
                    break;
                case '\r':
                    output << "\\r";
                    break;
                case '\t':
                    output << "\\t";
                    break;
                default:
                    if (character < 0x20U)
                    {
                        output << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(character) << std::dec;
                    }
                    else
                    {
                        output << character;
                    }
                }
            }
            output << '"';
            return output.str();
        }

        std::string error_json(const std::string &code, const std::string &message)
        {
            return "{\"status\":\"error\",\"code\":" + quote_json(code) + ",\"message\":" + quote_json(message) + '}';
        }

        class MissionPlannerNode final : public rclcpp::Node
        {
        public:
            MissionPlannerNode() : Node("capability_mission_planner")
            {
                const auto max_request_bytes = declare_parameter<int64_t>("max_request_bytes", 1024 * 1024);
                const auto max_result_bytes = declare_parameter<int64_t>("max_result_bytes", 4 * 1024 * 1024);
                if (max_request_bytes <= 0 || max_result_bytes <= 0)
                    throw std::runtime_error("message size limits must be positive");
                _max_request_bytes = static_cast<std::size_t>(max_request_bytes);
                _max_result_bytes = static_cast<std::size_t>(max_result_bytes);

                _service = create_service<rcl_interfaces::srv::SetParameters>(
                    "/capability_mission_planner/plan",
                    std::bind(&MissionPlannerNode::handle_plan, this, std::placeholders::_1, std::placeholders::_2));
                RCLCPP_INFO(get_logger(), "ready for mission requests");
            }

        private:
            using SetParameters = rcl_interfaces::srv::SetParameters;

            void finish(const SetParameters::Response::SharedPtr &response, bool successful, std::string reason) const
            {
                response->results.resize(1U);
                response->results.front().successful = successful;
                response->results.front().reason = std::move(reason);
            }

            void handle_plan(const SetParameters::Request::SharedPtr request, SetParameters::Response::SharedPtr response)
            {
                try
                {
                    if (request->parameters.size() != 1U)
                    {
                        finish(response, false, error_json("INVALID_REQUEST", "exactly one mission_json string parameter is required"));
                        return;
                    }
                    const auto &parameter = request->parameters.front();
                    if (parameter.name != "mission_json" || parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                    {
                        finish(response, false, error_json("INVALID_REQUEST", "parameter mission_json must have type PARAMETER_STRING"));
                        return;
                    }
                    const auto &payload = parameter.value.string_value;
                    if (payload.empty() || payload.size() > _max_request_bytes)
                    {
                        finish(response, false, error_json("REQUEST_TOO_LARGE", "mission_json is empty or exceeds max_request_bytes"));
                        return;
                    }

                    const auto request_root = YAML::Load(payload);
                    if (!request_root.IsMap() || !request_root["request_id"] ||
                        !request_root["request_id"].IsScalar() || !request_root["map"] ||
                        !request_root["map"].IsMap() || !request_root["map"]["directory"] ||
                        !request_root["map"]["directory"].IsScalar())
                    {
                        finish(response, false, error_json("INVALID_REQUEST", "request_id and map.directory are required"));
                        return;
                    }
                    const auto request_id = request_root["request_id"].as<std::string>();
                    const auto map_directory = std::filesystem::path(request_root["map"]["directory"].as<std::string>());
                    std::error_code filesystem_error;
                    const bool map_directory_exists = std::filesystem::is_directory(map_directory, filesystem_error);
                    const auto cache_directory = request_root["map"]["cache_directory"];
                    const bool invalid_cache_directory = cache_directory && (!cache_directory.IsScalar() || !std::filesystem::path(cache_directory.as<std::string>()).is_absolute());
                    if (request_id.empty() || !map_directory.is_absolute() || !map_directory_exists || filesystem_error || invalid_cache_directory)
                    {
                        const auto message = request_id.empty() ? "request_id must not be empty" : (invalid_cache_directory ? "map.cache_directory must be an absolute path" : "map.directory must be an existing absolute directory");
                        finish(response, false, error_json("INVALID_REQUEST", message));
                        return;
                    }

                    ConfiguredMission mission;
                    try
                    {
                        mission = OfflinePlannerConfigLoader::load_node(request_root);
                    }
                    catch (const std::exception &error)
                    {
                        finish(response, false, error_json("INVALID_REQUEST", error.what()));
                        return;
                    }

                    OfflineMissionPlan plan;
                    try
                    {
                        OfflineMissionPlanner planner(MultiMapPathPlanner(mission.bundle, mission.traversal), mission.objective);
                        plan = planner.plan(mission.robots, mission.tasks, mission.coordinate_conflicts);
                    }
                    catch (const std::exception &error)
                    {
                        finish(response, false, error_json("PLANNING_FAILED", error.what()));
                        return;
                    }
                    const auto plan_json = PlanExporter::to_json(*mission.bundle, mission.robots, mission.tasks, plan, mission.export_options);
                    std::string response_json = "{\"request_id\":" + quote_json(request_id) + ",\"status\":\"success\",\"plan\":" + plan_json + '}';
                    if (response_json.size() > _max_result_bytes)
                    {
                        finish(response, false, error_json("RESULT_TOO_LARGE", "plan result exceeds max_result_bytes"));
                        return;
                    }
                    finish(response, true, std::move(response_json));
                    RCLCPP_INFO(get_logger(), "completed request %s using map directory %s", request_id.c_str(), map_directory.c_str());
                }
                catch (const YAML::Exception &error)
                {
                    finish(response, false, error_json("INVALID_REQUEST", error.what()));
                }
                catch (const std::exception &error)
                {
                    finish(response, false, error_json("PLANNING_FAILED", error.what()));
                }
            }

            std::size_t _max_request_bytes = 0;
            std::size_t _max_result_bytes = 0;
            rclcpp::Service<SetParameters>::SharedPtr _service;
        };
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<capability_mission_planner::offline::MissionPlannerNode>());
    }
    catch (const std::exception &error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("capability_mission_planner"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
