from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
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
    use_vision_arg = DeclareLaunchArgument(
        "use_vision",
        default_value="true",
        description="Запускать vision_node",
    )
    vision_yolo_model_arg = DeclareLaunchArgument("vision_yolo_model", default_value="yolov8n.pt")
    vision_yolo_imgsz_arg = DeclareLaunchArgument("vision_yolo_imgsz", default_value="640")
    vision_tracker_arg = DeclareLaunchArgument("vision_tracker", default_value="bytetrack.yaml")
    vision_enable_face_embedding_arg = DeclareLaunchArgument(
        "vision_enable_face_embedding",
        default_value="false",
    )
    vision_face_model_arg = DeclareLaunchArgument("vision_face_model", default_value="")
    vision_embedding_interval_arg = DeclareLaunchArgument("vision_embedding_interval", default_value="25")
    vision_face_min_confidence_arg = DeclareLaunchArgument("vision_face_min_confidence", default_value="0.5")
    vision_head_yaw_min_deg_arg = DeclareLaunchArgument("vision_head_yaw_min_deg", default_value="-45.0")
    vision_head_yaw_max_deg_arg = DeclareLaunchArgument("vision_head_yaw_max_deg", default_value="45.0")
    vision_head_yaw_scale_deg_arg = DeclareLaunchArgument("vision_head_yaw_scale_deg", default_value="57.2958")
    vision_face_crop_ratio_arg = DeclareLaunchArgument("vision_face_crop_ratio", default_value="0.45")
    vision_embedding_input_size_arg = DeclareLaunchArgument("vision_embedding_input_size", default_value="112")
    vision_embedding_norm_mean_arg = DeclareLaunchArgument("vision_embedding_norm_mean", default_value="127.5")
    vision_embedding_norm_std_arg = DeclareLaunchArgument("vision_embedding_norm_std", default_value="128.0")
    vision_yaw_eye_dx_epsilon_arg = DeclareLaunchArgument("vision_yaw_eye_dx_epsilon", default_value="1e-6")

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
    vision_node = Node(
        package="server",
        executable="vision_node",
        name="vision_node",
        output="screen",
        parameters=[{
            "yolo_model": LaunchConfiguration("vision_yolo_model"),
            "yolo_imgsz": LaunchConfiguration("vision_yolo_imgsz"),
            "tracker": LaunchConfiguration("vision_tracker"),
            "enable_face_embedding": LaunchConfiguration("vision_enable_face_embedding"),
            "face_model": LaunchConfiguration("vision_face_model"),
            "embedding_interval": LaunchConfiguration("vision_embedding_interval"),
            "face_min_confidence": LaunchConfiguration("vision_face_min_confidence"),
            "head_yaw_min_deg": LaunchConfiguration("vision_head_yaw_min_deg"),
            "head_yaw_max_deg": LaunchConfiguration("vision_head_yaw_max_deg"),
            "head_yaw_scale_deg": LaunchConfiguration("vision_head_yaw_scale_deg"),
            "face_crop_ratio": LaunchConfiguration("vision_face_crop_ratio"),
            "embedding_input_size": LaunchConfiguration("vision_embedding_input_size"),
            "embedding_norm_mean": LaunchConfiguration("vision_embedding_norm_mean"),
            "embedding_norm_std": LaunchConfiguration("vision_embedding_norm_std"),
            "yaw_eye_dx_epsilon": LaunchConfiguration("vision_yaw_eye_dx_epsilon"),
        }],
        condition=IfCondition(LaunchConfiguration("use_vision")),
    )

    return LaunchDescription([
        config_arg,
        transport_arg,
        use_vision_arg,
        vision_yolo_model_arg,
        vision_yolo_imgsz_arg,
        vision_tracker_arg,
        vision_enable_face_embedding_arg,
        vision_face_model_arg,
        vision_embedding_interval_arg,
        vision_face_min_confidence_arg,
        vision_head_yaw_min_deg_arg,
        vision_head_yaw_max_deg_arg,
        vision_head_yaw_scale_deg_arg,
        vision_face_crop_ratio_arg,
        vision_embedding_input_size_arg,
        vision_embedding_norm_mean_arg,
        vision_embedding_norm_std_arg,
        vision_yaw_eye_dx_epsilon_arg,
        node,
        vision_node,
    ])
