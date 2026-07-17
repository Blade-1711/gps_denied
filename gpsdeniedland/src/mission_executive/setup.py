import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mission_executive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Config files (mavros.yaml)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vikrant',
    maintainer_email='vikrant@todo.todo',
    description='Mission executive and test nodes for GPS-denied navigation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_executive_node = mission_executive.mission_executive_node:main',
            'takeoff_land_test = mission_executive.takeoff_land_test:main',
        ],
    },
)
