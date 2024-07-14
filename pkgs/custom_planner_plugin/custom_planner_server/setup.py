# setup.py

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'custom_planner_server'
planner_submodule = package_name + '/planners' 
rrt_submodule = planner_submodule + '/rrt'
rrt_star_submodule = planner_submodule + '/rrt_star'
dijkstra_submodule = planner_submodule + '/dijkstra'
a_star_submodule = planner_submodule + '/a_star'

pre_processing_submodule = package_name + '/pre_processing'
post_processing_submodule = package_name + '/post_processing'

utilites_submodule = package_name + '/utilities'


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, planner_submodule, rrt_submodule, rrt_star_submodule, dijkstra_submodule, a_star_submodule, pre_processing_submodule, post_processing_submodule,utilites_submodule, 'test'],  # Include 'test' directory '''planner_submodule,'''
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'node_args'],
    zip_safe=True,
    maintainer='Ryan McKee',
    maintainer_email='rmckee31@qub.ac.uk',
    description='ROS2 Python Custom Planner Server Package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_server = custom_planner_server.planner_server:main',
        ],
    },
)
