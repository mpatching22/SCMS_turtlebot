import os
from glob import glob
from setuptools import setup

package_name = 'depth_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models/depth_cam'), glob('models/depth_cam/*.sdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SCMS Team',
    maintainer_email='you@example.com',
    description='Depth-image based person follower for TurtleBot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'depth_follower_node = depth_follower.depth_follower_node:main',
        ],
    },
)
