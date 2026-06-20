from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aetherscan_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='AetherScan Team',
    maintainer_email='dev@aetherscan.io',
    description='SLAM pipeline for AetherScan drone',
    license='MIT',
    entry_points={
        'console_scripts': [
            'point_cloud_assembler = aetherscan_slam.point_cloud_assembler:main',
            'map_manager = aetherscan_slam.map_manager:main',
        ],
    },
)
