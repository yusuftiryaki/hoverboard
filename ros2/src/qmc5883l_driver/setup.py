from setuptools import find_packages, setup

package_name = "qmc5883l_driver"

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
    description="QMC5883L magnetometer driver publishing sensor_msgs/MagneticField.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "qmc5883l = qmc5883l_driver.mag_node:main",
        ],
    },
)
