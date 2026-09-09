#include <capability_mission_planner/offline_map_planner.hpp>
#include <capability_mission_planner/offline_planner_config.hpp>

#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <system_error>

namespace capability_mission_planner::offline
{
    namespace
    {
        class MissionPlannerNode final : public rclcpp::Node
        {
        public:
            MissionPlannerNode() : Node("capability_mission_planner")
            {
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
                        finish(response, false, "exactly one mission_yaml_path string parameter is required");
                        return;
                    }
                    const auto &parameter = request->parameters.front();
                    if (parameter.name != "mission_yaml_path" || parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                    {
                        finish(response, false, "parameter mission_yaml_path must have type PARAMETER_STRING");
                        return;
                    }
                    const auto config_path = std::filesystem::path(parameter.value.string_value);
                    if (config_path.empty() || !config_path.is_absolute() || config_path.extension() != ".yaml")
                    {
                        finish(response, false, "mission_yaml_path must be an existing absolute .yaml file");
                        return;
                    }
                    std::error_code filesystem_error;
                    if (!std::filesystem::is_regular_file(config_path, filesystem_error) || filesystem_error)
                    {
                        finish(response, false, "mission_yaml_path does not refer to a regular file");
                        return;
                    }

                    ConfiguredMission mission;
                    try
                    {
                        mission = OfflinePlannerConfigLoader::load(config_path);
                    }
                    catch (const std::exception &error)
                    {
                        finish(response, false, error.what());
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
                        finish(response, false, error.what());
                        return;
                    }
                    PlanExporter::write(mission.output_directory, *mission.bundle, mission.robots, mission.tasks, plan, mission.export_options);
                    const auto output_path = std::filesystem::absolute(mission.output_directory / "plan.json").lexically_normal();
                    finish(response, true, output_path.string());
                    RCLCPP_INFO(get_logger(), "completed mission %s, output %s", config_path.c_str(), output_path.c_str());
                }
                catch (const std::exception &error)
                {
                    finish(response, false, error.what());
                }
            }

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
