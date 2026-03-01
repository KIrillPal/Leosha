from setuptools import setup, find_packages


package_name = "server"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=["server", "server.*"]),
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
        "PyYAML>=6.0",
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
    entry_points={
        "console_scripts": [
            "server = server.main:main",
        ],
    },
)
