from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='capability_mission_planner',
            executable='capability_mission_planner_node',
            name='capability_mission_planner',
            output='screen',
        ),
    ])
