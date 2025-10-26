from setuptools import setup
import os
from glob import glob

package_name = 'my_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': ''},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vdtune',
    maintainer_email='voductuan1305@gmail.com',
    description='A package to avoid obstacles with a mobile robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoid_obstacle = my_avoidance.avoid_obstacle:main',
        ],
    },
)

