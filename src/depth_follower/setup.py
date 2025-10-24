from setuptools import setup
import os
from glob import glob

package_name = 'depth_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 13',
    maintainer_email='micah@example.com',
    description='RGB-D person following for TurtleBot3 Waffle Pi',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_follower = depth_follower.depth_follower:main'
        ],
    },
)
