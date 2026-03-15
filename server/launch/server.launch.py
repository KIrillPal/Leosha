import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

_VISION_REQUIRED_KEYS = (
    "yolo_model",
    "yolo_device",
    "yolo_conf",
    "yolo_imgsz",
    "tracker",
    "enable_face_embedding",
    "face_model",
    "embedding_interval",
    "face_min_confidence",
    "head_yaw_min_deg",
    "head_yaw_max_deg",
    "head_yaw_scale_deg",
    "face_crop_ratio",
    "embedding_input_size",
    "embedding_norm_mean",
    "embedding_norm_std",
    "yaw_eye_dx_epsilon",
    "log_level",
)


def _vision_params_from_config(context):
    config_path = LaunchConfiguration("config").perform(context)
    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Config {config_path}: root must be a YAML mapping")
    if "vision" not in data:
        raise ValueError(f"Config {config_path}: missing 'vision' section")
    vision = data["vision"]
    if not isinstance(vision, dict):
        raise ValueError(f"Config {config_path}: 'vision' must be a mapping")
    for key in _VISION_REQUIRED_KEYS:
        if key not in vision:
            raise ValueError(f"Config {config_path}: vision section missing required key '{key}'")
    return vision


def _vision_node_with_config(context, *args, **kwargs):
    vision = _vision_params_from_config(context)
    log_level = vision.pop("log_level")
    parameters = {k: v for k, v in vision.items()}
    return [
        Node(
            package="server",
            executable="vision_node",
            name="vision_node",
            output="screen",
            arguments=["--log-level", str(log_level)],
            parameters=[parameters],
            condition=IfCondition(LaunchConfiguration("use_vision")),
        ),
    ]


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
    use_vision_arg = DeclareLaunchArgument(
        "use_vision",
        default_value="true",
        description="Запускать vision_node",
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
    vision_node = OpaqueFunction(
        function=lambda context: _vision_node_with_config(context),
    )

    return LaunchDescription([
        config_arg,
        transport_arg,
        use_vision_arg,
        node,
        vision_node,
    ])
