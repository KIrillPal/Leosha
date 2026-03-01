from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=[FindPackageShare("server"), "/config/server.yaml"],
        description="Путь к YAML конфигу для server",
    )

    node = Node(
        package="server",
        executable="server",
        name="server",
        output="screen",
        arguments=["--config", LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, node])
