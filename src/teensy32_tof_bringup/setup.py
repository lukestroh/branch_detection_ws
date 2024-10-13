from setuptools import find_packages, setup
import os
import glob

package_name = 'teensy32_tof_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + find_packages(where="~/microROS-msgs-ws", exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*launch.py')),
        # Rviz files
        (os.path.join('share', package_name, 'rviz'), glob.glob('rviz/*.rviz')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lukestroh',
    maintainer_email='luke.strohbehn@gmail.com',
    description='Teensy3.2 Time-of-Flight bringup package',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tof = teensy32_tof_bringup.tof_processor:main",
        ],
    },
)
