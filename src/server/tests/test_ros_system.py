"""In-process bringup. Skipped unless LEO_RUN_ROS=1 inside a sourced Humble shell."""

import os

import pytest

pytestmark = pytest.mark.ros


def test_nodes_and_monitor_report_fps():
    if os.environ.get("LEO_RUN_ROS") != "1":
        pytest.skip("set LEO_RUN_ROS=1 in a sourced ROS Humble environment")
    import threading
    import time

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String

    from leo_server.common.config import load_config
    from leo_server.nodes.mode_manager.node import ModeManager
    from leo_server.nodes.monitor.node import Monitor
    from leo_server.nodes.odometry.node import OdometryNode
    from leo_server.nodes.robot_bridge.node import RobotBridge

    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    cfg = load_config(os.path.join(root, "config", "robot.yaml"))
    rclpy.init()
    nodes = [RobotBridge(cfg), ModeManager(cfg), OdometryNode(cfg), Monitor(cfg)]
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        time.sleep(1.5)
        names = {name for name, _ns in nodes[0].get_node_names_and_namespaces()}
        assert {"robot_bridge", "mode_manager", "odometry", "monitor"} <= names
        mode = nodes[1].get_parameter("mode").value
        assert mode == "pause"
        nodes[1].set_parameters([rclpy.parameter.Parameter("mode", value="teleop_slam")])
        time.sleep(0.3)
        assert nodes[1]._controller.uses_slam()
        nodes[1].set_parameters([rclpy.parameter.Parameter("mode", value="teleoperation")])
        assert nodes[1]._controller.uses_slam() is False
        nodes[1].set_parameters([rclpy.parameter.Parameter("mode", value="teleop_slam")])
        assert nodes[1]._controller.uses_slam() is True
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
        del String
