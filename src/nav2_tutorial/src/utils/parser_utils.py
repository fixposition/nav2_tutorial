import os
import copy
import glob
import math
import yaml
import rclpy
import argparse
from pyproj import Transformer
from typing import List, Tuple
from geometry_msgs.msg import PoseStamped
from src.utils.gps_utils import quaternion_from_euler, latLonYaw2Geopose
from ament_index_python.packages import get_package_share_directory


class YamlWaypointParser:
    """Parse a set of GPS waypoints from a YAML file."""

    def __init__(self, wps_file_path: str, base_lat: float = 0.0,
                 base_lon: float = 0.0, base_alt: float = 0.0):
        """Initialize the parser with a YAML file path and base coordinates.
        Args:
            wps_file_path (str): Path to the YAML file containing waypoints.
            base_lat (float): Latitude of the base point for ENU transformation.
            base_lon (float): Longitude of the base point for ENU transformation.
            base_alt (float): Altitude of the base point for ENU transformation.
        Raises:
            FileNotFoundError: If the specified YAML file does not exist.
            KeyError: If the 'waypoints' key is missing in the YAML file.
            RuntimeError: If the YAML file cannot be parsed.
        """
        if not os.path.isfile(wps_file_path):
            raise FileNotFoundError(wps_file_path)

        try:
            with open(wps_file_path, "r") as wps_file:
                self.wps_dict = yaml.safe_load(wps_file) or {}
        except yaml.YAMLError as err:
            raise RuntimeError(f"Failed to parse YAML file '{wps_file_path}': {err}")

        if "waypoints" not in self.wps_dict:
            raise KeyError(f"Key 'waypoints' missing from YAML file '{wps_file_path}'.")
        
        # Raise warning if base coordinates are not set
        if base_lat == 0.0 and base_lon == 0.0 and base_alt == 0.0:
            print("Warning: Base coordinates are set to (0.0, 0.0, 0.0). "
                  "This may lead to incorrect ENU transformations. "
                  "Please set appropriate base coordinates for your map frame.")
        
        # Transform from WGS-84 (lat, lon, h) to ENU (e, n, u) in the map frame
        self._update_base(base_lat, base_lon, base_alt)
        
    def _update_base(self, base_lat: float, base_lon: float, base_alt: float):
        """Update the ENU transformation base point."""
        enu_pipeline = (
            "+proj=pipeline "
            "+step +proj=unitconvert +xy_in=deg +xy_out=rad "
            "+step +proj=axisswap +order=2,1,3 "  # lon,lat order
            "+step +proj=cart +ellps=WGS84 "
            f"+step +proj=topocentric +ellps=WGS84 "
            f"      +lat_0={base_lat} +lon_0={base_lon} +h_0={base_alt} "
        )
        self._tf_llh2enu = Transformer.from_pipeline(enu_pipeline)

    @staticmethod
    def _reverse_yaw(yaw: float) -> float:
        """Return yaw rotated by pi, wrapped to [-pi, pi]."""
        yaw += math.pi
        return (yaw + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _dedup(wps: List[dict]) -> List[dict]:
        """Filter duplicate waypoints that are closer than ~1 cm."""
        if not wps:
            return []
        cleaned = [wps[0]]
        for wp in wps[1:]:
            dx = wp["latitude"]  - cleaned[-1]["latitude"]
            dy = wp["longitude"] - cleaned[-1]["longitude"]
            if abs(dx) > 1e-8 or abs(dy) > 1e-8:   # ~1 cm in lat/lon
                cleaned.append(wp)
        return cleaned
    
    def _latlon_to_pose(self, lat: float, lon: float, yaw: float) -> PoseStamped:
        """Convert latitude, longitude, and yaw to a PoseStamped in the map frame."""
        x, y, _ = self._tf_llh2enu.transform(lat, lon, 0.0)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp    = rclpy.time.Time().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
        return pose

    def get_wps(self, reverse: bool = False) -> List[dict]:
        """Return a list of waypoints as PoseStamped objects.
        Args:
            reverse (bool): If True, reverse the order of waypoints.
        Returns:
            List[dict]: A list of waypoints as PoseStamped objects.
        """
        # Extract waypoints from the loaded YAML dictionary
        waypoints = copy.deepcopy(self.wps_dict["waypoints"])
        if reverse:
            waypoints.reverse()
            for wp in waypoints:
                wp["yaw"] = self._reverse_yaw(wp["yaw"])
            
        # Filter out duplicate waypoints
        waypoints = self._dedup(waypoints)
        
        return [self._latlon_to_pose(wp["latitude"], wp["longitude"], wp["yaw"]) for wp in waypoints]


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
    """Return (src_traj_dir, share_traj_dir), *share_traj_dir* is the directory 
    installed by ``ament`` (typically ``install/share/nav2_tutorial/trajectories``)
    while *src_traj_dir* points to the editable source tree.
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
        chosen = None
        src_latest = _latest_file(src_traj_dir)
        share_latest = _latest_file(share_traj_dir)

        if src_latest and share_latest:
            # Compare by timestamp embedded in filename (lexical order) or mtime
            chosen = src_latest if os.path.basename(src_latest) > os.path.basename(share_latest) else share_latest
        else:
            chosen = src_latest or share_latest  # whichever is not None (could still be None)

        if chosen is None:
            raise FileNotFoundError(
                f"No waypoint files matching 'gps_waypoints_2*.yaml' found in '{src_traj_dir}' or '{share_traj_dir}'."
            )
        return chosen

    # 3) Default lookup "gps_waypoints.yaml"
    default_name = "gps_waypoints.yaml"
    for base in (src_traj_dir, share_traj_dir):
        candidate = os.path.join(base, default_name)
        if os.path.isfile(candidate):
            return candidate

    raise FileNotFoundError(
        f"Default waypoint file '{default_name}' not found in '{src_traj_dir}' or '{share_traj_dir}'."
    )
