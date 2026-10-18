from setuptools import find_packages, setup

package_name = "robot_learning"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Enes",
    maintainer_email="enesis@entes.com.tr",
    description="Learning mode: record episodes (rosbag2/MCAP) for imitation learning.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "episode_recorder = robot_learning.recorder_node:main",
            "image_compressor = robot_learning.image_compressor:main",
            "episode_export = robot_learning.export:main",
        ],
    },
)
