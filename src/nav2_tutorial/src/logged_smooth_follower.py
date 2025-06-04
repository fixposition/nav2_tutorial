import os
import sys
import time
import math
import yaml
import glob
import argparse
from datetime import datetime

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from ament_index_python.packages import get_package_share_directory

from src.utils.gps_utils import latLonYaw2Geopose

from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL  


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """
    Returns (x, y, z, w).
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _latest_file(directory: str, pattern: str = "gps_waypoints_2*.yaml") -> str | None:
    """Return the lexically newest file matching *pattern* inside *directory*.

    The files produced by *terminal_logger.py* embed a timestamp in their name
    (e.g. ``gps_waypoints_20250317_142311.yaml``) so lexical order ===
    chronological order. If *directory* does not exist or contains no matching
    files, ``None`` is returned.
    """
    if not os.path.isdir(directory):
        return None
    files = glob.glob(os.path.join(directory, pattern))
    if not files:
        return None
    files.sort()
    return files[-1]


def _resolve_ws_paths() -> tuple[str, str]:
    """Return ``(src_traj_dir, share_traj_dir)`` for *nav2_tutorial*.

    *share_traj_dir* is the directory installed by ``ament`` (typically
    ``install/share/nav2_tutorial/trajectories``) while *src_traj_dir* points to
    the editable source tree (``<ws_root>/src/nav2_tutorial/trajectories``).
    """
    share_dir = get_package_share_directory("nav2_tutorial")
    share_traj_dir = os.path.join(share_dir, "trajectories")

    # share/nav2_tutorial -> share -> install -> <ws_root>
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
    src_traj_dir = os.path.join(ws_root, "src", "nav2_tutorial", "trajectories")
    return src_traj_dir, share_traj_dir


def _select_yaml(args: argparse.Namespace) -> str:
    """Figure out which YAML file we should load.

    Priority:
    1. If the user passed an explicit path, use it.
    2. If ``--last`` was requested, compare the newest file from the *src* and
       *share* directories and choose the newer one (with a preference for the
       *src* file if both have identical timestamps).
    3. Otherwise, attempt to load ``gps_waypoints.yaml`` from *src* first then
       *share*.

    A *FileNotFoundError* is raised if no suitable file is found so that the
    caller can decide how to handle the error.
    """
    src_traj_dir, share_traj_dir = _resolve_ws_paths()

    # 1) Explicit path
    if args.yaml_file is not None:
        explicit = os.path.expanduser(args.yaml_file)
        if not os.path.isfile(explicit):
            raise FileNotFoundError(explicit)
        return explicit

    # 2) --last flag
    if args.last:
        src_latest = _latest_file(src_traj_dir)
        share_latest = _latest_file(share_traj_dir)

        if src_latest and share_latest:
            # Compare by timestamp embedded in filename (lexical order) or mtime
            chosen = src_latest if os.path.basename(src_latest) > os.path.basename(share_latest) else share_latest
        else:
            chosen = src_latest or share_latest  # whichever is not None (could still be None)

        if chosen is None:
            raise FileNotFoundError(f"No waypoint files matching 'gps_waypoints_2*.yaml' found in '{src_traj_dir}' or '{share_traj_dir}'.")
        return chosen

    # 3) Default lookup "gps_waypoints.yaml"
    default_name = "gps_waypoints.yaml"
    candidate_src = os.path.join(src_traj_dir, default_name)
    if os.path.isfile(candidate_src):
        return candidate_src

    candidate_share = os.path.join(share_traj_dir, default_name)
    if os.path.isfile(candidate_share):
        return candidate_share

    raise FileNotFoundError(f"Default waypoint file '{default_name}' not found in '{src_traj_dir}' or '{share_traj_dir}'.")


class YamlWaypointParser:
    """Parse a set of GPS waypoints from a YAML file."""

    def __init__(self, wps_file_path: str):
        if not os.path.isfile(wps_file_path):
            raise FileNotFoundError(wps_file_path)

        try:
            with open(wps_file_path, "r") as wps_file:
                self.wps_dict = yaml.safe_load(wps_file) or {}
        except yaml.YAMLError as err:
            raise RuntimeError(f"Failed to parse YAML file '{wps_file_path}': {err}")

        if "waypoints" not in self.wps_dict:
            raise KeyError(f"Key 'waypoints' missing from YAML file '{wps_file_path}'.")

    @staticmethod
    def _reverse_yaw(yaw: float) -> float:
        """Return yaw rotated by π, wrapped to [-π, π]."""
        new_yaw = yaw + math.pi
        # Wrap to [-π, π]
        return (new_yaw + math.pi) % (2 * math.pi) - math.pi

    def get_wps(self, reverse: bool = False):
        """Return a list of ``GeoPose`` objects, possibly in reverse order."""
        waypoints = self.wps_dict["waypoints"][::-1] if reverse else self.wps_dict["waypoints"]
        gepose_wps = []
        for wp in waypoints:
            yaw = self._reverse_yaw(wp["yaw"]) if reverse else wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(wp["latitude"], wp["longitude"], yaw))
        return gepose_wps


class GpsWpCommander:
    """Follow a smooth sequence of GPS waypoints."""

    def __init__(self, wps_file_path: str, reverse: bool = False):
        self.navigator  = BasicNavigator("basic_navigator")
        self.wp_parser  = YamlWaypointParser(wps_file_path)
        self.reverse    = reverse

        # FromLL client (WGS-84 → map frame) – create it on the navigator node
        self._fromll_cli = self.navigator.create_client(FromLL, "/fromLL")
        self.navigator.get_logger().info("Waiting for /fromLL service…")
        if not self._fromll_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("robot_localization /fromLL service is not available")

    # Convert (lat, lon, yaw) to PoseStamped in the map frame
    def _latlon_to_pose(self, lat: float, lon: float, yaw: float) -> PoseStamped:
        req = FromLL.Request()
        req.ll_point.latitude  = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude  = 0.0

        future = self._fromll_cli.call_async(req)
        rclpy.spin_until_future_complete(self.navigator, future)
        map_pt = future.result().map_point

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp    = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = map_pt.x
        pose.pose.position.y = map_pt.y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw       = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def start_ntp(self):
        """Send one NavigateThroughPoses goal and block until done."""
        self.navigator.waitUntilNav2Active(localizer="robot_localization")

        raw_wps = self.wp_parser.wps_dict["waypoints"]
        if self.reverse:
            raw_wps = raw_wps[::-1]

        poses = []
        for wp in raw_wps:
            yaw = (self.wp_parser._reverse_yaw(wp["yaw"])
                   if self.reverse else wp["yaw"])
            poses.append(self._latlon_to_pose(
                wp["latitude"], wp["longitude"], yaw))

        self.navigator.goThroughPoses(poses)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.1)

        print("Waypoints completed successfully (continuous mode)")


def main(argv: list[str] | None = None):
    rclpy.init(args=argv)

    parser = argparse.ArgumentParser(description="Follow GPS waypoints from a YAML file")
    parser.add_argument("yaml_file", nargs="?", default=None, help="Path to the YAML waypoints file")
    parser.add_argument("--last", action="store_true", help="Load the most recent waypoints file in the config directory")
    parser.add_argument("--reverse", action="store_true", help="Follow the trajectory in reverse order (adds 180° to yaw)")
    args = parser.parse_args()

    try:
        yaml_file_path = _select_yaml(args)
    except FileNotFoundError as e:
        print(f"[WARN] {e}", file=sys.stderr)
        sys.exit(1)

    # Feedback to the user
    print(f"Loading waypoints file: {yaml_file_path}")

    try:
        gps_wpf = GpsWpCommander(yaml_file_path, reverse=args.reverse)
        gps_wpf.start_ntp()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
