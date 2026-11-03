"""Hide ROS launch's own screen lines for a LEO launch.

``ros2 launch`` prints its verbosity banner before the launch file is loaded.
This option extension runs first and raises only the screen handler to
``CRITICAL``. ``launch.log`` keeps the same records.
"""

from __future__ import annotations

import logging


def is_leo_launch(package_name: str, launch_file_name: str) -> bool:
    """True when this ``ros2 launch`` invocation starts LEO."""
    package = package_name or ""
    launch_file = launch_file_name or ""
    if package == "leo_server":
        return True
    return package.endswith("leo.launch.py") or launch_file.endswith("leo.launch.py")


def quiet_screen() -> None:
    """Stop the launch screen handler from printing. Leave file handlers unchanged."""
    from launch.logging import launch_config

    launch_config.get_screen_handler().setLevel(logging.CRITICAL)


def _option_base():
    try:
        from ros2launch.option import OptionExtension
    except ImportError:
        return object
    return OptionExtension


class QuietLaunchScreen(_option_base()):
    """``ros2 launch`` option that quiets the screen before the banner is logged."""

    EXTENSION_POINT_VERSION = "0.1"

    def prestart(self, args):
        package = "" if args is None else str(getattr(args, "package_name", "") or "")
        launch_file = "" if args is None else str(getattr(args, "launch_file_name", "") or "")
        if is_leo_launch(package, launch_file):
            quiet_screen()
