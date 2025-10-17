from setuptools import setup, find_packages
import os

def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

model_files = package_files('models')

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/warehouse_env']),
    ('share/warehouse_env/launch', ['launch/warehouse_world.launch.py']),
    ('share/warehouse_env/worlds', ['worlds/warehouse.world.sdf']),
    ('share/warehouse_env', ['package.xml']),
]

for f in model_files:
    rel = os.path.relpath(os.path.dirname(f), '.')
    data_files.append((f'share/warehouse_env/{rel}', [f]))

setup(
    name='warehouse_env',
    version='0.0.1',
    packages=find_packages(),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Micah Patching',
    maintainer_email='micah@petruscorp.com',
    description='Gazebo warehouse world, models, and actor animation',
    license='BSD-3-Clause',
)
