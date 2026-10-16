from setuptools import find_packages, setup

package_name = "robot_missions"

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
    description="The site (areas, keep-outs, station) and the missions run on it.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "site_map_server = robot_missions.site_map_server:main",
            "mission_runner = robot_missions.mission_runner:main",
        ],
    },
)
