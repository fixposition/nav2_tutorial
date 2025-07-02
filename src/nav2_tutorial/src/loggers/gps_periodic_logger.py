#!/usr/bin/env python3
from __future__ import annotations
import os
import sys
import tty
import math
import time
import yaml
import select
import signal
import termios
import argparse
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from ament_index_python.packages import get_package_share_directory

from src.utils.gps_utils import euler_from_quaternion

# Global variables
stop_requested = False

class GpsPeriodicLogger(Node):
    def __init__(self, logging_file_path: str, interval: float):
        super().__init__('gps_waypoint_periodic_logger')

        self.logging_file_path = logging_file_path
        self.log_interval = interval
        self.last_gps: NavSatFix | None = None
        self.last_yaw: float = 0.0
        self.saved_points: int = 0
        self.waypoints: list[dict] = []
        self.flush_threshold = 10

        self._DIST_TOL_M = 0.01     # Save only if robot moved more than 1 cm
        self._DEG2M_LAT = 110_574.0 # Rough conversion at mid-latitudes
        self._last_saved_fix: NavSatFix | None = None

        self.create_subscription(NavSatFix, '/fixposition/odometry_llh', self.gps_callback, qos_profile=1)
        self.create_subscription(Odometry, '/fixposition/odometry_enu', self.yaw_callback, qos_profile=1)

        self.create_timer(self.log_interval, self._log_waypoint)
        self.create_timer(2.0, self._status_message)

        self.get_logger().info(
            f"Logging every {self.log_interval:.2f} s to '{self.logging_file_path}'. Press 'q' to stop."
        )

    def gps_callback(self, msg: NavSatFix) -> None:
        self.last_gps = msg

    def yaw_callback(self, msg: Odometry) -> None:
        _, _, self.last_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def _log_waypoint(self) -> None:
        fix = self.last_gps
        if fix is None:
            return

        if not (-90 <= fix.latitude <= 90 and -180 <= fix.longitude <= 180):
            return

        if self.saved_points == 0:
            self.get_logger().info("Started collecting waypoints …")

        if self._last_saved_fix is not None:
            dlat = (fix.latitude - self._last_saved_fix.latitude) * self._DEG2M_LAT
            lon_scale = 111_320.0 * math.cos(math.radians(fix.latitude))
            dlon = (fix.longitude - self._last_saved_fix.longitude) * lon_scale
            if math.hypot(dlat, dlon) < self._DIST_TOL_M:
                return

        self._last_saved_fix = fix

        waypoint = {
            'latitude': fix.latitude,
            'longitude': fix.longitude,
            'altitude': fix.altitude,
            'yaw': self.last_yaw,
            'logged_at': datetime.now().isoformat()
        }

        self.waypoints.append(waypoint)
        self.saved_points += 1

        if len(self.waypoints) >= self.flush_threshold:
            self.flush_to_disk()

    def flush_to_disk(self) -> None:
        try:
            with open(self.logging_file_path, 'r') as fh:
                data = yaml.safe_load(fh) or {}
        except FileNotFoundError:
            data = {}
        except Exception as exc:
            self.get_logger().error(f"Failed to read log file: {exc}")
            return

        existing = data.get('waypoints', [])
        existing.extend(self.waypoints)
        data['waypoints'] = existing

        tmp_path = self.logging_file_path + ".tmp"
        try:
            with open(tmp_path, 'w') as fh:
                yaml.dump(data, fh, default_flow_style=False)
            os.replace(tmp_path, self.logging_file_path)
            self.get_logger().info(f"Flushed {len(self.waypoints)} waypoint(s) to disk.")
            self.waypoints.clear()
        except Exception as exc:
            self.get_logger().error(f"Failed to write log file: {exc}")

    def _status_message(self) -> None:
        if self.saved_points == 0:
            return

        self.get_logger().info(
            f"Total waypoints saved: {self.saved_points} | "
            f"Last -> lat={self.last_gps.latitude:.6f}, "
            f"lon={self.last_gps.longitude:.6f}, yaw={self.last_yaw:.2f}"
        )

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
    parser.add_argument('-i', '--interval', type=float, default=0.2, help='Log interval in seconds (default: 0.2).')
    args = parser.parse_args(argv)
    return args.yaml or _default_yaml_path(), args.interval

def _stdin_ready() -> bool:
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def handle_sigterm(signum, frame):
    global stop_requested
    stop_requested = True

def main(argv: list[str] | None = None) -> None:
    yaml_path, interval = _parse_arguments(argv)

    rclpy.init(args=argv)
    node = GpsPeriodicLogger(yaml_path, interval)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    global stop_requested
    stop_requested = False
    signal.signal(signal.SIGTERM, handle_sigterm)

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
            if _stdin_ready():
                ch = sys.stdin.read(1)
                if ch == 'q':
                    node.get_logger().info("'q' pressed - stopping waypoint recording.")
                    break
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user, shutting down.")
    finally:
        if node.waypoints:
            node.get_logger().info("Saving remaining waypoints before exit...")
            node.flush_to_disk()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
