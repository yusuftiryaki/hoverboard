from setuptools import find_packages, setup

package_name = "battery_manager"

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
    description="Battery monitoring, state-of-charge estimation and return-to-dock.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "battery_monitor = battery_manager.battery_monitor_node:main",
            "battery_supervisor = battery_manager.battery_supervisor_node:main",
        ],
    },
)