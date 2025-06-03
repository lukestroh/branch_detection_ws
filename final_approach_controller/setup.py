from setuptools import find_packages, setup

import os
import glob

package_name = "final_approach_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Launch files
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lukestroh",
    maintainer_email="luke.strohbehn@gmail.com",
    description="Final approach controller package",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "final_approach_controller = final_approach_controller.final_approach_controller:main",
            "cut_point_rotate_axis_controller = final_approach_controller.cut_point_rotate_axis_controller:main",
            "find_branch_roll_wrist_controller = final_approach_controller.find_branch_roll_wrist_controller:main",
            "generate_poses_service = final_approach_controller.generate_poses_service:main",
            "reset_test = final_approach_controller.reset_test:main",
        ],
    },
)
