from __future__ import annotations

import os
import sys
import tty
import math
import time
import yaml
import select
import termios
import argparse
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from ament_index_python.packages import get_package_share_directory

from src.utils.gps_utils import euler_from_quaternion


class GpsKeyLogger(Node):
    """
    ROS2 node to log GPS waypoints to a YAML file on keypress.
    Press 'f' to log, 'q' to quit.
    """

    def __init__(self, logging_file_path: str):
        super().__init__('gps_waypoint_keylogger')

        self.logging_file_path = logging_file_path
        self.last_gps: NavSatFix | None = None
        self.last_yaw = 0.0
        self.last_press_time = 0.0
        self.saved_points: int = 0
        self.waypoints: list[dict] = []
        self.flush_threshold = 10
        
        self._DIST_TOL_M = 0.01     # Save only if robot moved more than 1 cm
        self._DEG2M_LAT = 110_574.0 # Rough conversion at mid-latitudes
        self._last_saved_fix: NavSatFix | None = None
        self._last_header_ns: int | None = None  # Last header stamp [ns]

        self.gps_topic = '/fixposition/odometry_llh'
        self.odom_topic = '/fixposition/odometry_enu'

        self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, 1)
        self.create_subscription(Odometry, self.odom_topic, self.yaw_callback, 1)

        self.get_logger().info(
            f"Logging to '{self.logging_file_path}'.\n"
            f"Subscribed to GPS: {self.gps_topic}, Odometry: {self.odom_topic}.\n"
             "Press 'f' to log a waypoint; 'q' to quit."
        )

    def gps_callback(self, msg: NavSatFix) -> None:
        self.last_gps = msg

    def yaw_callback(self, msg: Odometry) -> None:
        _, _, self.last_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def log_waypoint(self) -> None:
        fix = self.last_gps
        if fix is None:
            self.get_logger().warn("No GPS fix yet; skipping log.")
            return

        # Reject out-of-range coordinates
        if not (-90.0 <= fix.latitude <= 90.0 and -180.0 <= fix.longitude <= 180.0):
            self.get_logger().warn("Received invalid GPS coordinates.")
            return

        # Reject out‑of‑order fixes (header time going backwards)
        stamp = fix.header.stamp
        curr_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self._last_header_ns is not None and curr_ns < self._last_header_ns:
            delta_us = (self._last_header_ns - curr_ns) / 1_000
            self.get_logger().warn(
                f"Skipping out-of-order fix: header_time went backwards by {delta_us:.0f} µs")
            return

        if self.saved_points == 0:
            self.get_logger().info("Started collecting waypoints …")
            
        # Distance gate to avoid duplicates / jitter
        if self._last_saved_fix is not None:
            dlat = (fix.latitude - self._last_saved_fix.latitude) * self._DEG2M_LAT
            lon_scale = 111_320.0 * math.cos(math.radians(fix.latitude))
            dlon = (fix.longitude - self._last_saved_fix.longitude) * lon_scale
            if math.hypot(dlat, dlon) < self._DIST_TOL_M:
                return

        # Store LLH point
        self._last_saved_fix = fix
        self._last_header_ns = curr_ns
        fix_time_iso = datetime.utcfromtimestamp(curr_ns * 1e-9).isoformat()

        wp = {
            'latitude':  fix.latitude,
            'longitude': fix.longitude,
            'altitude':  fix.altitude,
            'yaw':       self.last_yaw,
            'header_time': fix_time_iso,
            'logged_at': datetime.now().isoformat()
        }
        
        self.waypoints.append(wp)
        self.saved_points += 1

        if len(self.waypoints) >= self.flush_threshold:
            self.flush_to_disk()
            
        self.get_logger().info(
            f"Logged waypoint #{len(self.waypoints)}: "
            f"lat={wp['latitude']:.6f}, lon={wp['longitude']:.6f}, yaw={wp['yaw']:.2f}"
        )

    def flush_to_disk(self) -> None:
        try:
            with open(self.logging_file_path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            data = {}
        except Exception as e:
            self.get_logger().error(f"Failed to read log file: {e}")
            return

        existing = data.get('waypoints', [])
        existing.extend(self.waypoints)
        data['waypoints'] = existing

        tmp_path = f"{self.logging_file_path}.tmp"
        try:
            with open(tmp_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            os.replace(tmp_path, self.logging_file_path)
            self.get_logger().info(f"Flushed {len(self.waypoints)} waypoint(s) to disk.")
            self.waypoints.clear()
        except Exception as e:
            self.get_logger().error(f"Failed to write log file: {e}")


def _default_yaml_path() -> str:
    share_dir = get_package_share_directory('nav2_tutorial')
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
    src_traj = os.path.join(ws_root, 'src', 'nav2_tutorial', 'trajectories')
    target_dir = src_traj if os.path.isdir(src_traj) else os.path.join(share_dir, 'trajectories')
    os.makedirs(target_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(target_dir, f'gps_waypoints_{ts}.yaml')


def _parse_arguments(argv: list[str] | None = None) -> tuple[str, float]:
    parser = argparse.ArgumentParser(description="Periodic GPS Waypoint Logger")
    parser.add_argument('yaml', nargs='?', help='Path to the YAML log file.')
    args = parser.parse_args(argv)
    return args.yaml or _default_yaml_path()


def _stdin_ready():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def main(argv=None):
    yaml_path = _parse_arguments(argv)
    
    rclpy.init(args=argv)
    node = GpsKeyLogger(yaml_path)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if _stdin_ready():
                now = time.time()
                c = sys.stdin.read(1)
                if c == 'f' and (now - node.last_press_time > 0.2):
                    node.last_press_time = now
                    node.log_waypoint()
                elif c == 'q':
                    node.get_logger().info("Quit key pressed, shutting down.")
                    break
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user, shutting down.")
    finally:
        if node.waypoints:
            node.get_logger().info("Saving remaining waypoints before exit...")
            node.flush_to_disk()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
