import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'questions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
        # Install SDF files to the share/questions/worlds directory
        (os.path.join('share', package_name, 'worlds'), glob('*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya-roy',
    maintainer_email='adityaroy.jpg@gmail.com',
    description='interplanter',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'websocket_broadcaster = questions.websocket_broadcaster:main',
            'waypoint_navigator = questions.waypoint_navigator:main',
            'vision_node = questions.vision_node:main'
        ],
    },
)
