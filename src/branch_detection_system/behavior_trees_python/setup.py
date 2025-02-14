from setuptools import find_packages, setup
import os
import glob

package_name = "behavior_trees_python"

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
    description="PyTrees implementation",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fa_tree_node = behavior_trees_python.tree:main",
            "teleop_node = behavior_trees_python.teleop_node:main",
            "io_tree_node = behavior_trees_python.io_tree:main",
            "io_manager_node = behavior_trees_python.io_manager:main",
            "set_point_service_node = behavior_trees_python.set_point_service:main",
            "set_point_from_endpoint_service_node = behavior_trees_python.set_point_from_endpoint_service:main"
        ],
    },
)
