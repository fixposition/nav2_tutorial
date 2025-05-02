import os
import sys
import time
import yaml
import argparse
import glob

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from ament_index_python.packages import get_package_share_directory

from src.utils.gps_utils import latLonYaw2Geopose


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWpCommander:
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path: str):
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = YamlWaypointParser(wps_file_path)

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        wps = self.wp_parser.get_wps()
        self.navigator.followGpsWaypoints(wps)
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        print("Wps completed successfully")


def main():
    rclpy.init()

    parser = argparse.ArgumentParser(
        description="Follow GPS waypoints from a YAML file"
    )
    parser.add_argument(
        'yaml_file', nargs='?', default=None,
        help='Path to the YAML waypoints file'
    )
    parser.add_argument(
        '--last', action='store_true',
        help='Load the most recent waypoints file in the config directory'
    )
    args = parser.parse_args()

    config_dir = os.path.join(
        get_package_share_directory('nav2_tutorial'), 'config'
    )

    if args.last:
        pattern = os.path.join(config_dir, 'gps_waypoints_*.yaml')
        files = glob.glob(pattern)
        if not files:
            print(f"No files matching {pattern}", file=sys.stderr)
            sys.exit(1)
        # filenames with timestamps sort lexically
        files.sort()
        yaml_file_path = files[-1]
        print(f"Loading most recent waypoints file: {yaml_file_path}")
    else:
        if args.yaml_file:
            yaml_file_path = args.yaml_file
        else:
            yaml_file_path = os.path.join(
                config_dir, 'gps_waypoints.yaml'
            )
        print(f"Loading waypoints file: {yaml_file_path}")

    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == '__main__':
    main()
