from setuptools import setup
import os
from glob import glob

package_name = 'my_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Louis LE LAY',
    maintainer_email='le.lay.louis@gmail.com',
    description='ROS2 package for obstacle avoidance using LiDAR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoid_obstacle = my_avoidance.avoid_obstacle:main',
        ],
    },
)
