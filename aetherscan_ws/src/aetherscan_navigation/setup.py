from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aetherscan_navigation'

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
    description='Navigation and exploration for AetherScan',
    license='MIT',
    entry_points={
        'console_scripts': [
            'frontier_explorer = aetherscan_navigation.frontier_explorer:main',
            'path_planner = aetherscan_navigation.path_planner:main',
            'obstacle_avoidance = aetherscan_navigation.obstacle_avoidance:main',
        ],
    },
)
