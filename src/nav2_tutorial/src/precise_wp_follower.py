#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import yaml
from typing import List, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from pyproj import Transformer
from src.utils.gps_utils import quaternion_from_euler
from src.utils.parser_utils import YamlWaypointParser, _latest_file, _resolve_ws_paths, _select_yaml

# ROS msgs
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data


class GpsWpCommander:
    def __init__(self, yaml_path: str, *, reverse: bool = False):
        self.navigator = BasicNavigator("basic_navigator")
        self.wps       = YamlWaypointParser(yaml_path).get_wps(reverse=reverse)
        if not self.wps:
            raise RuntimeError("Waypoint list is empty.")

        # Build ENU transformer ---------------------------------------
        base_lat, base_lon, base_alt = self._get_datum()
        enu_pipeline = (
            "+proj=pipeline "
            "+step +proj=unitconvert +xy_in=deg +xy_out=rad "
            "+step +proj=axisswap +order=2,1,3 "
            "+step +proj=cart +ellps=WGS84 "
            f"+step +proj=topocentric +ellps=WGS84 +lat_0={base_lat} +lon_0={base_lon} +h_0={base_alt} "
        )
        self._tf_llh2enu = Transformer.from_pipeline(enu_pipeline)

        self.max_retries = 1

    # --------------------- datum helper ------------------------------
    def _get_datum(self, topic: str = "/fixposition/datum", timeout_s: float = 3.0):
        datum = None
        def _cb(msg: NavSatFix):
            nonlocal datum
            datum = (msg.latitude, msg.longitude, msg.altitude)
        sub = self.navigator.create_subscription(NavSatFix, topic, _cb, qos_profile_sensor_data)
        deadline = self.navigator.get_clock().now() + rclpy.duration.Duration(seconds=timeout_s)
        while rclpy.ok() and datum is None:
            if self.navigator.get_clock().now() > deadline:
                raise RuntimeError(f"No NavSatFix on {topic} within {timeout_s}s")
            rclpy.spin_once(self.navigator, timeout_sec=0.05)
        self.navigator.destroy_subscription(sub)
        return datum

    # --------------------- conversion helper -------------------------
    def _latlon_to_pose(self, lat: float, lon: float, yaw: float) -> PoseStamped:
        x, y, _ = self._tf_llh2enu.transform(lat, lon, 0.0)
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = rclpy.time.Time().to_msg()
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.position.z = 0.0
        ps.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
        return ps

    # --------------------- main loop ---------------------------------
    def start(self):
        self.navigator.waitUntilNav2Active(localizer="robot_localization")
        poses = [self._latlon_to_pose(wp["latitude"], wp["longitude"], wp["yaw"]) for wp in self.wps]
        self.navigator.get_logger().info(f"Starting precise run with {len(poses)} waypoint(s)…")

        for idx, pose in enumerate(poses, start=1):
            retries_left = self.max_retries
            while True:
                self.navigator.goToPose(pose)
                # spin until done or shutdown
                while rclpy.ok() and not self.navigator.isTaskComplete():
                    rclpy.spin_once(self.navigator, timeout_sec=0.05)
                result = self.navigator.getResult()
                # -------------------------------------------------- SUCCEEDED
                if result == TaskResult.SUCCEEDED:
                    self.navigator.get_logger().info(f"Waypoint {idx}/{len(poses)} reached")
                    break
                # -------------------------------------------------- FAILED / CANCELED
                if result in (TaskResult.FAILED, TaskResult.CANCELED):
                    if retries_left > 0:
                        retries_left -= 1
                        self.navigator.get_logger().warn(
                            f"Waypoint {idx} {result.name.lower()} - retrying ({self.max_retries - retries_left}/{self.max_retries})")
                        continue
                    self.navigator.get_logger().error(f"Waypoint {idx} failed twice - aborting trajectory")
                    return
                # -------------------------------------------------- other / unknown
                self.navigator.get_logger().error(f"Unexpected TaskResult {result} - aborting")
                return

        self.navigator.get_logger().info("All waypoints completed successfully!")


def main(argv: List[str] | None = None):
    rclpy.init(args=argv)
    parser = argparse.ArgumentParser(description="Follow GPS waypoints from a YAML file")
    parser.add_argument("yaml_file", nargs="?", default=None, help="Path to YAML waypoint file")
    parser.add_argument("--last", action="store_true", help="Use the most recent waypoint file")
    parser.add_argument("--reverse", action="store_true", help="Follow the path in reverse order (adds 180° yaw)")
    args = parser.parse_args()

    try:
        yaml_path = _select_yaml(args)
    except FileNotFoundError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)

    try:
        commander = GpsWpCommander(yaml_path, reverse=args.reverse)
        commander.start()
    except KeyboardInterrupt:
        print("Interrupted by user - shutting down…", file=sys.stderr)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
