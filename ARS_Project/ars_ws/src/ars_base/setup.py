import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ars_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ty',
    maintainer_email='zhechengg20@gmail.com',
    description='ARS mecanum base: Arduino serial bridge, URDF, and launch files.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = ars_base.serial_bridge:main',
            'safety_guard = ars_base.safety_guard:main',
            'obstacle_verifier = ars_base.obstacle_verifier:main',
            'teleop_keys = ars_base.teleop_keys:main',
            'event_logger = ars_base.event_logger:main',
        ],
    },
)
