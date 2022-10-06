from setuptools import setup

PACKAGE_NAME = "kinemic_ros_bridge"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
    ],
    install_requires=["setuptools", "kserviceconnect"],
    zip_safe=True,
    tests_require=[],
    entry_points={
        "console_scripts": [
            "kinemic_ros_node = kinemic_ros_bridge.kinemic_ros_node:main"
        ],
    },
    dependency_links=[
        "https://repo.kinemic.com/repository/pypi/simple/kserviceconnect/",
        "https://repo.kinemic.com/repository/pypi/simple/sensorproto/",
    ],
)
