from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agilex_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes/scout_v2'), glob('meshes/scout_v2/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Facundo Garcia',
    maintainer_email='facundo.garcia@fixposition.com',
    description='GPS Waypoints Follower for AgileX Scout Mini',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logged_waypoint_follower = agilex_demo.logged_waypoint_follower:main',
            'interactive_waypoint_follower = agilex_demo.interactive_waypoint_follower:main',
            'gps_waypoint_logger = agilex_demo.gps_waypoint_logger:main'
        ],
    },
)
