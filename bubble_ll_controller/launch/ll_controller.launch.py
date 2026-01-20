from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("bubble_ll_controller"), "config", "ll_controller.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="bubble_ll_controller",
                executable="ll_controller.py",
                name="ll_controller",
                output="screen",
                parameters=[
                    {"config_path": config_path}
                ],
            )
        ]
    )
