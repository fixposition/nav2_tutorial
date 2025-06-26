#!/usr/bin/env python3
"""
Periodic GPS waypoint logger for ROS2
==============================================================

* Saves a waypoint every *interval* seconds (default: 0.2s).
* Logs when logging starts (first waypoint collected).
* Prints a throttled status message containing the most recent waypoint every 2s.
* Press **q** in the console to stop recording gracefully.

Usage examples::

    ros2 run <your_pkg> gps_periodic_logger.py                 # default
    ros2 run <your_pkg> gps_periodic_logger.py -i 0.1          # faster
    ros2 run <your_pkg> gps_periodic_logger.py logs/points.yaml
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import tty
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

import yaml
from ament_index_python.packages import get_package_share_directory

from src.utils.gps_utils import euler_from_quaternion


class GpsPeriodicLogger(Node):
    """ROS2 node that periodically logs GPS waypoints to a YAML file."""

    def __init__(self, logging_file_path: str, interval: float):
        super().__init__('gps_waypoint_periodic_logger')

        self.logging_file_path = logging_file_path
        self.log_interval = interval
        self.last_gps: NavSatFix | None = None
        self.last_yaw: float = 0.0
        self.saved_points: int = 0
        
        self._DIST_TOL_M   = 0.01       # Save only if moved ≥ 1 cm
        self._DEG2M_LAT    = 110_574.0  # Rough conversion at mid-latitudes
        self._last_saved_fix   : NavSatFix | None = None

        # ------------------------------------------------------------------
        # Topic subscriptions
        # ------------------------------------------------------------------
        self.create_subscription(
            NavSatFix,
            '/fixposition/odometry_llh',
            self.gps_callback,
            qos_profile=1,
        )
        self.create_subscription(
            Odometry,
            '/fixposition/odometry_enu',
            self.yaw_callback,
            qos_profile=1,
        )

        # ------------------------------------------------------------------
        # Timers: waypoint logger + throttled status at 2 s
        # ------------------------------------------------------------------
        self.create_timer(self.log_interval, self._log_waypoint)
        self.create_timer(2.0, self._status_message)

        self.get_logger().info(
            f"Will log a waypoint every {self.log_interval:.3f} s to '{self.logging_file_path}'."
        )

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------
    def gps_callback(self, msg: NavSatFix) -> None:
        self.last_gps = msg

    def yaw_callback(self, msg: Odometry) -> None:
        _, _, self.last_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _log_waypoint(self) -> None:
        """Append the current pose to YAML."""
        fix = self.last_gps
        if fix is None:
            return  # wait for first fix
        
        if self.saved_points == 0:
            # First time we succeed in logging – announce start of collection.
            self.get_logger().info("Started collecting waypoints …")
        
        if self._last_saved_fix is not None:
            dlat = (fix.latitude  - self._last_saved_fix.latitude ) * self._DEG2M_LAT
            # convert longitude degrees to metres at current latitude
            lon_scale = 111_320.0 * math.cos(math.radians(fix.latitude))
            dlon = (fix.longitude - self._last_saved_fix.longitude) * lon_scale
            dist = math.hypot(dlat, dlon)
            if dist < self._DIST_TOL_M:
                return  # too close → skip this sample
        
        # Passed the distance test → remember it for next time
        self._last_saved_fix = fix

        waypoint = {
            'latitude': self.last_gps.latitude,
            'longitude': self.last_gps.longitude,
            'yaw': self.last_yaw,
        }

        # Read existing file
        try:
            with open(self.logging_file_path, 'r') as fh:
                data: dict = yaml.safe_load(fh) or {}
        except FileNotFoundError:
            data = {}
        except Exception as exc:
            self.get_logger().error(f"Failed to read log file: {exc}")
            return

        waypoints = data.get('waypoints', [])
        waypoints.append(waypoint)
        data['waypoints'] = waypoints

        # Write back updated YAML
        try:
            with open(self.logging_file_path, 'w') as fh:
                yaml.dump(data, fh, default_flow_style=False)
        except Exception as exc:
            self.get_logger().error(f"Failed to write log file: {exc}")
            return

        self.saved_points += 1

    def _status_message(self) -> None:
        """Emit a throttled info log with the most recently recorded waypoint."""
        if self.saved_points == 0:
            return

        self.get_logger().info(
            f"Total waypoints saved: {self.saved_points} | "
            f"Last -> lat={self.last_gps.latitude:.6f}, "
            f"lon={self.last_gps.longitude:.6f}, yaw={self.last_yaw:.2f}"
        )


def _default_yaml_path() -> str:
    share_dir = get_package_share_directory('nav2_tutorial')

    # share/nav2_tutorial ➜ share ➜ install ➜ ws root
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))

    src_traj = os.path.join(ws_root, 'src', 'nav2_tutorial', 'trajectories')
    target_dir = src_traj if os.path.isdir(src_traj) else os.path.join(share_dir, 'trajectories')
    os.makedirs(target_dir, exist_ok=True)

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(target_dir, f'gps_waypoints_{ts}.yaml')


def _parse_arguments(argv: list[str] | None = None) -> tuple[str, float]:
    parser = argparse.ArgumentParser(description="Periodic GPS waypoint logger")
    parser.add_argument('yaml', nargs='?', help='Path to the YAML log file.')
    parser.add_argument('-i', '--interval', type=float, default=0.2, help='Save interval in seconds (default 0.2).')
    args = parser.parse_args(argv)
    yaml_path = args.yaml or _default_yaml_path()
    return yaml_path, args.interval


def _stdin_ready() -> bool:
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def main(argv: list[str] | None = None) -> None:
    yaml_path, interval = _parse_arguments(argv)

    rclpy.init(args=argv)
    node = GpsPeriodicLogger(yaml_path, interval)

    # Configure terminal for non‑blocking single‑character input
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while rclpy.ok():
            # Spin (but return regularly so we can poll the keyboard)
            rclpy.spin_once(node, timeout_sec=0.1)

            if _stdin_ready():
                ch = sys.stdin.read(1)
                if ch == 'q':
                    node.get_logger().info("'q' pressed - stopping waypoint recording.")
                    break
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user, shutting down.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
