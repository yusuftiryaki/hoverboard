from setuptools import find_packages, setup

package_name = "mpu6050_driver"

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
    description="MPU6050 6-axis IMU driver publishing sensor_msgs/Imu.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpu6050 = mpu6050_driver.imu_node:main",
        ],
    },
)
