from glob import glob
from setuptools import find_packages, setup


package_name = "three_sensor_overlay"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        (
            "share/" + package_name + "/urdf",
            glob("urdf/*.urdf") + glob("urdf/*.xacro"),
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Antobot calibration",
    maintainer_email="noreply@example.com",
    description="Three-sensor LiDAR/camera replay overlay for ROS 2 and RViz.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sensor_overlay_node = three_sensor_overlay.sensor_overlay_node:main",
        ],
    },
)
