# depth_follower/setup.py
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
        ('share/' + package_name + '/launch', ['launch/follower.launch.py']),
        ('share/' + package_name + '/config', ['config/follower.params.yaml']),
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
