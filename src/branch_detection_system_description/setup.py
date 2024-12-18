from setuptools import find_packages, setup

import os
import glob

import rclpy.logging

logger = rclpy.logging.get_logger("setup.py")
import pprint as pp

package_name = "branch_detection_system_description"

urdf_relative_base_path = "urdf"
urdf_files = glob.glob("urdf/**/*.xacro", recursive=True)
mesh_relative_base_path = "meshes"
obj_files = glob.glob("meshes/**/*.obj", recursive=True)
stl_files = glob.glob("meshes/**/*.STL", recursive=True)
mesh_files = obj_files + stl_files

_data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    # (os.path.join("share", package_name, "urdf"), glob.glob("urdf/*.urdf", recursive=True)),
    (os.path.join("share", package_name, "config"), glob.glob("config/*.yaml")),
    (os.path.join("share", package_name, "rviz"), glob.glob("rviz/*.rviz")),
]
# Keep file structures
# URDF
for file in urdf_files:
    relative_path = os.path.relpath(file, urdf_relative_base_path)
    install_path = os.path.join("share", package_name, urdf_relative_base_path, os.path.dirname(relative_path))
    _data_files.append((install_path, [file]))
# Mesh
for file in mesh_files:
    relative_path = os.path.relpath(file, mesh_relative_base_path)
    install_path = os.path.join("share", package_name, mesh_relative_base_path, os.path.dirname(relative_path))
    _data_files.append((install_path, [file]))
    # logger.warn(f"{install_path}")
# logger.warn(f'{pp.pformat(_data_files)}')

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=_data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lukestroh",
    maintainer_email="luke.strohbehn@gmail.com",
    description="Description package for the complete system. Allows users to dynamically construct pruning robots",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
