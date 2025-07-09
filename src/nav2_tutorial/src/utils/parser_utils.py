import os
import glob
import yaml
import argparse
from typing import List, Tuple
from src.utils.gps_utils import latLonYaw2Geopose
from ament_index_python.packages import get_package_share_directory


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

    def get_wps(self, reverse: bool = False) -> List[dict]:
        """Return a list of ``GeoPose`` objects, possibly in reverse order."""
        waypoints = self.wps_dict["waypoints"][::-1] if reverse else self.wps_dict["waypoints"]
        waypoints = self._dedup(waypoints)
        
        
        
        
        geopose_wps = []
        for wp in waypoints:
            yaw = self._reverse_yaw(wp["yaw"]) if reverse else wp["yaw"]
            geopose_wps.append(latLonYaw2Geopose(wp["latitude"], wp["longitude"], yaw))
        return geopose_wps


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
