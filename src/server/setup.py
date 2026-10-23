from glob import glob
import os

from setuptools import find_packages, setup

package_name = "leo_server"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/leo_server"]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "robot_bridge = leo_server.nodes.robot_bridge.node:main",
            "mode_manager = leo_server.nodes.mode_manager.node:main",
            "odometry = leo_server.nodes.odometry.node:main",
            "vision = leo_server.nodes.vision.node:main",
            "web_ui = leo_server.nodes.web_ui.node:main",
            "monitor = leo_server.nodes.monitor.node:main",
        ],
    },
)
