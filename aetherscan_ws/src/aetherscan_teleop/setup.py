from setuptools import find_packages, setup

package_name = 'aetherscan_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='AetherScan Team',
    maintainer_email='dev@aetherscan.io',
    description='Keyboard teleoperation for AetherScan',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = aetherscan_teleop.keyboard_teleop:main',
        ],
    },
)
