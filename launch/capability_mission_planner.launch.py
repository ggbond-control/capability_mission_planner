from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_request_bytes', default_value='1048576'),
        DeclareLaunchArgument('max_result_bytes', default_value='4194304'),
        Node(
            package='capability_mission_planner',
            executable='capability_mission_planner_node',
            name='capability_mission_planner',
            output='screen',
            parameters=[{
                'max_request_bytes': ParameterValue(LaunchConfiguration('max_request_bytes'), value_type=int),
                'max_result_bytes': ParameterValue(LaunchConfiguration('max_result_bytes'), value_type=int),
            }],
        ),
    ])
