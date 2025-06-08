from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'eyes_v1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
    	'setuptools',
    	'eyes_msgs',
    ],
    zip_safe=True,
    maintainer='luka',
    maintainer_email='lukadalbello@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'robot_eyes_v1 = eyes_v1.robot_eyes_v1:main',
        	'pathfinder_a_star = eyes_v1.pathfinder_a_star:main',
        	'a_star_testing = eyes_v1.a_star_testing:main',
        	'a_star_simulator = eyes_v1.a_star_simulator:main',
        ],
    },
)
