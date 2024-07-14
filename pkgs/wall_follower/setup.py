from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'wall_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryan McKee',
    maintainer_email='rmckee31@qub.ac.uk',
    description='This is an experiemental package for wall following using a turtlebot3 under a specific set of circumstances',
    license='NA',
    entry_points={
        'console_scripts': [
            "wall_follower=wall_follower.wall_follower:main"
        ],
    },
)
