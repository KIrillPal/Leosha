from __future__ import annotations

import logging
import threading

LOGGER = logging.getLogger(__name__)


class Ros2ServerBridge:
    """ROS2 bridge: node, /scan publisher for slam_toolbox, SLAM subscriptions."""

    def __init__(
        self,
        slam_service=None,
        robot_client=None,
        odom_confidence_threshold: float = 0.5,
        get_control_mode=None,
        is_slam_active=None,
    ) -> None:
        self._enabled = False
        self._node = None
        self._slam_service = slam_service
        self._robot_client = robot_client
        self._odom_confidence_threshold = odom_confidence_threshold
        self._get_control_mode = get_control_mode
        self._is_slam_active = is_slam_active
        self._scan_pub = None
        self._scan_timer = None
        self._spin_thread = None
        self._spin_stop = threading.Event()

    def start(self) -> None:
        import rclpy
        from rclpy.node import Node
        rclpy.init(args=None)
        self._node = Node("server_bridge")
        self._enabled = True
        if self._slam_service:
            self._slam_service.set_ros_enabled(True)
            self._slam_service.try_subscribe_ros(self._node)
        if self._robot_client:
            self._start_scan_publisher()
        self._spin_stop.clear()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        LOGGER.info("ROS2 bridge started")

    def _start_scan_publisher(self) -> None:
        """Publish lidar scans to /scan and TF (odom->base_link, base_link->laser_frame) for slam_toolbox."""
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

        self._scan_pub = self._node.create_publisher(LaserScan, "/scan", 10)
        self._tf_broadcaster = TransformBroadcaster(self._node)
        self._static_tf = StaticTransformBroadcaster(self._node)

        # base_link -> laser_frame (static, lidar at robot origin)
        now = self._node.get_clock().now().to_msg()
        for parent, child in [("base_link", "laser_frame"), ("base_link", "base_footprint")]:
            static = TransformStamped()
            static.header.stamp = now
            static.header.frame_id = parent
            static.child_frame_id = child
            static.transform.translation.x = 0.0
            static.transform.translation.y = 0.0
            static.transform.translation.z = 0.0
            static.transform.rotation.x = 0.0
            static.transform.rotation.y = 0.0
            static.transform.rotation.z = 0.0
            static.transform.rotation.w = 1.0
            self._static_tf.sendTransform(static)

        self._scan_timer = self._node.create_timer(0.1, self._publish_scan)  # 10 Hz
        self._tf_timer = self._node.create_timer(0.02, self._publish_odom_tf)  # 50 Hz
        LOGGER.info("Scan publisher and TF (base_link->laser_frame, odom->base_link) started")

    def _get_odom_pose(self) -> tuple[float, float, float]:
        """(x, y, yaw) for odom->base_link: from SLAM if odom_confidence < threshold, else from telemetry."""
        telem = self._robot_client.get_latest_telemetry()
        if (
            self._slam_service and 
            telem.odom_confidence < self._odom_confidence_threshold
        ):
            x, y, theta = self._slam_service.get_pose()
            return (float(x), float(y), float(theta))
        return (float(telem.odom_x), float(telem.odom_y), float(telem.odom_yaw))

    def _is_teleop_slam(self) -> bool:
        """Whether SLAM feed should be active for current profile/mode."""
        if self._is_slam_active is not None:
            return bool(self._is_slam_active())
        if self._get_control_mode is None:
            return False
        from .models import ControlMode
        return self._get_control_mode() == ControlMode.TELEOP_SLAM

    def _publish_odom_tf(self) -> None:
        """Publish odom->base_link from robot odometry or SLAM pose (required by slam_toolbox)."""
        if not self._is_teleop_slam():
            return
        if not self._robot_client or not getattr(self, "_tf_broadcaster", None):
            return
        from geometry_msgs.msg import TransformStamped
        import math
        x, y, yaw = self._get_odom_pose()
        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf_broadcaster.sendTransform(t)

    def _publish_scan(self) -> None:
        if not self._is_teleop_slam():
            return
        scan = self._robot_client.get_latest_lidar_scan() if self._robot_client else None
        if scan is None:
            return
        from sensor_msgs.msg import LaserScan
        import math
        stamp = self._node.get_clock().now().to_msg()
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "laser_frame"
        ranges = scan["ranges"]
        angle_min = float(scan["angle_min"])
        angle_max = float(scan["angle_max"])
        n = len(ranges)
        # slam_toolbox/Karto expects len(ranges) == round((angle_max - angle_min) / angle_increment) + 1.
        # Ensure consistency to avoid "LaserRangeScan contains X range readings, expected Y".
        if n > 1:
            msg.angle_increment = (angle_max - angle_min) / (n - 1)
        else:
            msg.angle_increment = float(scan["angle_increment"])
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.range_min = float(scan["range_min"])
        msg.range_max = float(scan["range_max"])
        msg.ranges = [float(r) for r in ranges]
        intensities = scan["intensities"]
        if intensities:
            msg.intensities = [float(i) if i is not None else 0.0 for i in intensities]
        self._scan_pub.publish(msg)
        # Publish odom TF with same stamp as scan (slam_toolbox looks up at scan time)
        if getattr(self, "_tf_broadcaster", None):
            from geometry_msgs.msg import TransformStamped
            x, y, yaw = self._get_odom_pose()
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)
            self._tf_broadcaster.sendTransform(t)

    def _spin_loop(self) -> None:
        import rclpy
        while not self._spin_stop.is_set():
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def stop(self) -> None:
        if not self._enabled:
            return
        self._spin_stop.set()
        if self._spin_thread:
            self._spin_thread.join(timeout=2.0)
        import rclpy
        if self._node is not None:
            self._node.destroy_node()
        rclpy.shutdown()
        LOGGER.info("ROS2 bridge stopped")
