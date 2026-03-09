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

    transport_arg = DeclareLaunchArgument(
        "transport",
        default_value="",
        description="Транспорт до робота: mock / zmq (пусто = из конфига)",
    )

    node = Node(
        package="server",
        executable="server",
        name="server",
        output="screen",
        arguments=[
            "--config", LaunchConfiguration("config"),
            "--transport", LaunchConfiguration("transport"),
        ],
    )

    return LaunchDescription([config_arg, transport_arg, node])
