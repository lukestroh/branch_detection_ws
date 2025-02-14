from setuptools import find_packages, setup
import glob
import os

package_name = "pruning_admittance_controller"

setup(
    name=package_name,
    author=["lukestroh", "Hannah Kolano"],
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Launch files
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*launch.py")),
        # Config files
        (os.path.join("share", package_name, "config"), glob.glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lukestroh",
    maintainer_email="luke.strohbehn@gmail.com",
    description="admittance controller package",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wrench_filter_node = pruning_admittance_controller.wrench_filter:main",
            "contact_watcher_node = pruning_admittance_controller.contact_watcher:main",
            # "fake_wrench_pub_node = pruning_admittance_controller.fake_wrench_publisher:main",
            "admittance_controller_node = pruning_admittance_controller.admittance_controller:main",
        ],
    },
)
