from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_custom_planner_bringup'

def list_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), list_files('launch')),  # Include launch files
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps/tb3_world'), list_files('maps/tb3_world')),  # Include map files
        (os.path.join('share', package_name, 'models'), list_files('models')),  # Include model files
        (os.path.join('share', package_name, 'models/robots/turtlebot3_burger'), list_files('models/robots/turtlebot3_burger')),  # Include model files
        (os.path.join('share', package_name, 'models/worlds'), list_files('models/worlds')),  # Include model files
        (os.path.join('share', package_name, 'models/worlds/turtlebot3_world'), list_files('models/worlds/turtlebot3_world')),  # Include model files
        (os.path.join('share', package_name, 'models/worlds/turtlebot3_world/meshes'), list_files('models/worlds/turtlebot3_world/meshes')),  # Include model files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),  # Include rviz files
        (os.path.join('share', package_name, 'urdf'), list_files('urdf')),  # Include urdf files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='rmckee31@qub.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)