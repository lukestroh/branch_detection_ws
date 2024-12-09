from setuptools import find_packages, setup

import os
import glob

package_name = 'branch_detection_system_moveit_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*launch.py")),
        # Rviz files
        (os.path.join('share', package_name, 'rviz'), glob.glob('rviz/*.rviz')),
        # # Config files
        (os.path.join("share", package_name, "config"), glob.glob("config/*.yaml")),
        # (os.path.join("share", package_name, "urdf"), glob.glob("urdf/*.urdf", recursive=True)),
        (os.path.join("share", package_name, "srdf"), glob.glob("srdf/*.srdf")),
        (os.path.join("share", package_name, "srdf"), glob.glob("srdf/*.xacro")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lukestroh',
    maintainer_email='luke.strohbehn@gmail.com',
    description='MoveIt2 package for branch detection system',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
