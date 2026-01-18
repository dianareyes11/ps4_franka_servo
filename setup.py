from setuptools import setup
import os
from glob import glob

package_name = 'ps4_franka_servo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diana',
    maintainer_email='diana@example.com',
    description='PS4 controller interface for Franka Panda robot arm using MoveIt Servo for real-time control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps4_franka_servo = ps4_franka_servo.ps4_franka_servo:main',
        ],
    },
)
