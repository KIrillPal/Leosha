"""Launch server + slam_toolbox for teleop-slam profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("server"), "/launch/server.launch.py",
        ]),
        launch_arguments={
            "config": LaunchConfiguration("config"),
            "transport": LaunchConfiguration("transport"),
        }.items(),
    )

    ld = LaunchDescription([config_arg, transport_arg, server_launch])

    # Optional: include slam_toolbox (requires slam_toolbox package)
    try:
        from launch.conditions import IfCondition
        from ament_index_python.packages import get_package_share_directory
        import os

        use_slam_arg = DeclareLaunchArgument(
            "use_slam",
            default_value="true",
            description="Запускать slam_toolbox",
        )
        slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("slam_toolbox"), "/launch/online_async_launch.py",
            ]),
            launch_arguments={
                "use_sim_time": "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_slam")),
        )
        ld.add_action(use_slam_arg)
        ld.add_action(slam_launch)
    except Exception:
        pass  # slam_toolbox not installed

    return ld
