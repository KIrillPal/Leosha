import shutil
from pathlib import Path

from setuptools import setup, find_packages
from setuptools.command.install_scripts import install_scripts


package_name = "server"
_configs_dir = Path(__file__).resolve().parent.parent / "configs"

_local_config = Path(__file__).resolve().parent / "config" / "server.yaml"
if _configs_dir.joinpath("server.yaml").exists():
    _local_config.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(_configs_dir / "server.yaml", _local_config)


class install_scripts_with_env_shebang(install_scripts):
    """После установки подменяем shebang на #!/usr/bin/env python3 для conda/venv."""

    def run(self):
        super().run()
        for path in self.outfiles:
            with open(path, "r", encoding="utf-8") as f:
                lines = f.readlines()
            if lines and lines[0].startswith("#!") and "env" not in lines[0]:
                lines[0] = "#!/usr/bin/env python3\n"
                with open(path, "w", encoding="utf-8") as f:
                    f.writelines(lines)


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=["server", "server.*"]),
    package_data={"server": ["web/templates/*.html"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/templates", [
            "server/web/templates/base.html",
            "server/web/templates/settings.html",
            "server/web/templates/teleop.html",
            "server/web/templates/network.html",
        ]),
        (f"share/{package_name}/config", ["config/server.yaml"]),
        (f"share/{package_name}/launch", ["launch/server.launch.py"]),
    ],
    install_requires=[
        "setuptools",
        "Flask>=2.2",
        "numpy>=1.24",
        "Pillow>=9.0",
        "PyYAML>=6.0",
        "pyzmq>=25.0",
        "msgpack>=1.0",
    ],
    extras_require={
        "test": [
            "pytest>=7.0",
        ]
    },
    zip_safe=False,
    maintainer="Leosha Dev",
    maintainer_email="dev@example.com",
    description="ROS2 сервер управления роботом с YAML-конфигом, web GUI и mock-клиентом.",
    license="MIT",
    tests_require=["pytest"],
    scripts=["scripts/server"],
    cmdclass={"install_scripts": install_scripts_with_env_shebang},
)
