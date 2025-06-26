from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_tutorial'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'trajectories'), glob('trajectories/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
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
            'logged_waypoint_follower = src.logged_waypoint_follower:main',
            'logged_smooth_follower = src.logged_smooth_follower:main',
            'logged_refill_follower = src.logged_refill_follower:main',
            'simple_follower = src.simple_follower:main',
            'interactive_waypoint_follower = src.interactive_waypoint_follower:main',
            'gps_waypoint_logger = src.gps_waypoint_logger:main',
            'terminal_logger = src.terminal_logger:main',
            'periodic_logger = src.periodic_logger:main',
            'set_datum = src.set_datum:main'
        ],
    },
)
