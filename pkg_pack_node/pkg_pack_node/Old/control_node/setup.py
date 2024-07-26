
from glob import glob

from setuptools import setup

package_name = "control_node"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.launch.py")),
        ("share/" + package_name + "/configs", glob("configs/*.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Denis Štogl",
    author_email="denis@stogl.de",
    maintainer="Denis Štogl",
    maintainer_email="denis@stogl.de",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Demo nodes for showing and testing functionalities of ros2_control framework.",
    long_description="""\
Demo nodes for showing and testing functionalities of the ros2_control framework.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "publisher_gripper_controller = \
                control_node.publisher_gripper_controller:main",
        ],
    },
)