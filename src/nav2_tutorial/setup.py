import os
from glob import glob
from setuptools import find_packages, setup

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
    description='GPS Waypoint Follower for Vision-RTK2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # GPS Waypoint Follower
            'smooth_wp_follower = src.smooth_wp_follower:main',
            'precise_wp_follower = src.precise_wp_follower:main',
            'interactive_wp_follower = src.interactive_wp_follower:main',
            
            # GPS Waypoint Logger
            'gps_keylogger = src.loggers.gps_keylogger:main',
            'gps_periodic_logger = src.loggers.gps_periodic_logger:main',
            
            # Datum Setter
            'set_datum = src.utils.set_datum:main',
            'set_datum_from_tf = src.utils.set_datum_from_tf:main',
            
            # Visualizer
            'visualize_gps = src.utils.visualize_gps:main',
            
            # Dashboard
            'dashboard = src.dashboard.dashboard:main',
        ],
    },
)
