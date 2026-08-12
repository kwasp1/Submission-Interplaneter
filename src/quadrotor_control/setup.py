from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadrotor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
	(os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya-roy',
    maintainer_email='adityaroy.jpg@gmail.com',
    description='Package description',
    license='License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_control_node = quadrotor_control.voice_control_node:main',
            'telemetry_gui_node = quadrotor_control.telemetry_gui_node:main',
        ],
    },
)
