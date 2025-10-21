from setuptools import setup
from glob import glob
import os

package_name = 'depth_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('depth_follower/urdf/*.xacro')),  # fix path here
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Micah',
    maintainer_email='micah@todo.todo',
    description='Depth-following Turtlebot3 node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'depth_follower_node = depth_follower.depth_follower:main',
        ],
    },
)
