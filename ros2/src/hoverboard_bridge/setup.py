from setuptools import find_packages, setup

package_name = "hoverboard_bridge"

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
    description="ROS 2 <-> ESP32 serial bridge for the hoverboard drivetrain.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hoverboard_bridge = hoverboard_bridge.bridge_node:main",
            "fake_esp32 = hoverboard_bridge.fake_esp32:main",
        ],
    },
)
