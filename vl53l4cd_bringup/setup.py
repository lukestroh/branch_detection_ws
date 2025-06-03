from setuptools import find_packages, setup

import glob
import os

package_name = "vl53l4cd_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Launch files
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*launch.py")),
        # Config files
        (os.path.join("share", package_name, "config"), glob.glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob.glob("config/*.json")),
        # Plotjuggler
        (os.path.join("share", package_name, "plotjuggler"), glob.glob("plotjuggler/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lukestroh",
    maintainer_email="luke.strohbehn@gmail.com",
    description="vl53l4cd filter package",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vl53l4cd_filter_node = vl53l4cd_bringup.vl53l4cd_filter_node:main",
        ],
    },
)
