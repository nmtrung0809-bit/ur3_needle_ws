from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'ur3_needle_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Bắt buộc để ROS nhận package
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # File project
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'resource'), [
            f for f in glob('resource/*') if os.path.basename(f) != package_name
        ]),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abc',
    maintainer_email='nmtrung0809@gmail.com',
    description='UR3e needle insertion simulation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'needle_insert_node = ur3_needle_sim.needle_insert_node:main',
            'depth_monitor = ur3_needle_sim.depth_monitor:main',
        ],
    },
)
