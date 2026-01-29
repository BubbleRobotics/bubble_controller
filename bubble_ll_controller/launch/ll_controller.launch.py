from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("bubble_ll_controller"), "config", "ll_controller.yaml"]
    )

    

    use_bluetooth_arg  = DeclareLaunchArgument(
        'use_bluetooth',
        default_value='false',
        description='Bluetooth or not is the question'
    )

    use_bluetooth = LaunchConfiguration('use_bluetooth')

    node = Node(
                package="bubble_ll_controller",
                executable="ll_controller.py",
                name="ll_controller",
                output="screen",
                parameters=[
                    {"config_path": config_path},
                    {"use_bluetooth": use_bluetooth}
                ],
            )

    return LaunchDescription(
        [
            use_bluetooth_arg,
            node
        ]
    )
