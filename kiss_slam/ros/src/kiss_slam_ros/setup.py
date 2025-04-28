from setuptools import find_packages, setup

package_name = "kiss_slam_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nathan Hewitt",
    maintainer_email="mail@nhewitt-me",
    description="A simple ROS 2 wrapper package for KISS-SLAM",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["kiss_slam_node = kiss_slam_ros.kiss_slam_node:main"],
    },
)
