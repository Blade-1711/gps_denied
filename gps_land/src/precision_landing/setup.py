import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'precision_landing'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TerraWings',
    maintainer_email='terrawings@todo.todo',
    description='Activation-based precision landing with ArUco marker detection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'precision_landing_node = precision_landing.precision_landing_node:main',
            'camera_publisher = precision_landing.camera_publisher:main',
        ],
    },
)
