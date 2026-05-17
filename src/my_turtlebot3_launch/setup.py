from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'my_turtlebot3_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'laser_geometry'],
    zip_safe=True,
    maintainer='liuxingjian',
    maintainer_email='xliudt@connect.ust.hk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'laser_to_pointcloud = my_turtlebot3_launch.laser_to_pointcloud:main',
        ],
    },
)
