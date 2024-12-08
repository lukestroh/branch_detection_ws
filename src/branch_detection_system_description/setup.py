from setuptools import find_packages, setup

package_name = 'branch_detection_system_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lukestroh',
    maintainer_email='luke.strohbehn@gmail.com',
    description='Description package for the complete system. Allows users to dynamically construct pruning robots',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
