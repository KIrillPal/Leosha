from pathlib import Path

from setuptools import setup, find_packages


package_name = "client"
# configs лежат в code/configs (родитель setup.py = code/client, его родитель = code)
_configs_dir = Path(__file__).resolve().parent.parent / "configs"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=["client", "client.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        *([(f"share/{package_name}/config", [str(_configs_dir / "client.yaml")])] if _configs_dir.joinpath("client.yaml").exists() else []),
    ],
    install_requires=[
        "setuptools",
        "PyYAML>=6.0",
        "msgpack>=1.0",
        "numpy>=1.24",
        "pyzmq>=25.0",
        "Pillow>=9.0",
    ],
    extras_require={
        "test": [
            "pytest>=7.0",
        ]
    },
    zip_safe=False,
    maintainer="Leosha Dev",
    maintainer_email="dev@example.com",
    description="Клиент робота: потоки сенсоров, SensorHub, watchdog и сетевой мост.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "client = client.main:main",
        ],
    },
)
