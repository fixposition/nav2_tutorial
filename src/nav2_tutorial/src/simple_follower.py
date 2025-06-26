import os
import sys
import time
import math
import yaml
import glob
import argparse
from datetime import datetime

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory

from src.utils.gps_utils import latLonYaw2Geopose
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from uuid import UUID
from pyproj import CRS, Transformer

# ROS messages
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath

# Visualization
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
MARKER_VIZ = True


def _hsv_to_rgb(h, s=1.0, v=1.0):
    """h ∈ [0,1] → (r,g,b) ∈ [0,1]³  (simple HSV→RGB)"""
    i = int(h*6.0) % 6
    f = h*6.0 - i
    p, q, t = v*(1-s), v*(1-f*s), v*(1-(1-f)*s)
    if   i == 0: r,g,b = v,t,p
    elif i == 1: r,g,b = q,v,p
    elif i == 2: r,g,b = p,v,t
    elif i == 3: r,g,b = p,q,v
    elif i == 4: r,g,b = t,p,v
    else:        r,g,b = v,p,q
    return r, g, b


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
        """Return yaw rotated by pi, wrapped to [-pi, pi]."""
        new_yaw = yaw + math.pi
        return (new_yaw + math.pi) % (2 * math.pi) - math.pi

    def get_wps(self, reverse: bool = False):
        """Return a list of ``GeoPose`` objects, possibly in reverse order."""
        waypoints = self.wps_dict["waypoints"][::-1] if reverse else self.wps_dict["waypoints"]

        # Filter out immediate duplicates (distance < 1 cm)
        cleaned = [waypoints[0]]
        for wp in waypoints[1:]:
            dx = (wp["latitude"]  - cleaned[-1]["latitude"] )
            dy = (wp["longitude"] - cleaned[-1]["longitude"])
            if abs(dx) > 1e-8 or abs(dy) > 1e-8:      # ≈ 1 cm threshold
                cleaned.append(wp)
        waypoints = cleaned
        
        geopose_wps = []
        for wp in waypoints:
            yaw = self._reverse_yaw(wp["yaw"]) if reverse else wp["yaw"]
            geopose_wps.append(latLonYaw2Geopose(wp["latitude"], wp["longitude"], yaw))
        return geopose_wps


def _get_goal_handle(nav):
    """Return the ActionGoal handle of the *latest* NavigateThroughPoses goal."""
    for attr in ("_ntp_goal_handle",               # Iron, Jazzy
                 "_nav_through_poses_goal_handle"  # Humble, Galactic
                ):
        if hasattr(nav, attr):
            return getattr(nav, attr)
    return None            # very old / rolling-edge: no public handle

def _get_current_uuid(nav):
    """Return the UUID (bytes) of the goal that is *currently* active."""
    for attr in ("_current_nav_through_poses_uuid",  # Humble
                 "_current_ntp_uuid"                 # Iron/Jazzy
                ):
        if hasattr(nav, attr):
            return getattr(nav, attr)
    return None


class GpsWpCommander:
    """Follow a smooth sequence of GPS waypoints."""

    def __init__(self, wps_file_path: str, reverse: bool = False):
        self.navigator  = BasicNavigator("basic_navigator")
        self.wp_parser  = YamlWaypointParser(wps_file_path)
        self.reverse    = reverse

        # Odometry topic with ENU pose in map frame
        self._last_odom_pose: PoseStamped | None = None
        self.navigator.create_subscription(Odometry, "/fixposition/odometry_enu", self._odom_cb, 10)
        
        # Get datum
        _base_lat, _base_lon, _base_alt = self._get_datum()
        
        # Transform from WGS-84 (lat, lon, h) to ENU (e, n, u) in the map frame
        enu_pipeline = (
            "+proj=pipeline "
            "+step +proj=unitconvert +xy_in=deg +xy_out=rad "
            "+step +proj=axisswap +order=2,1,3 "               # lon,lat order for cart
            "+step +proj=cart +ellps=WGS84 "
            f"+step +proj=topocentric +ellps=WGS84 "
            f"      +lat_0={_base_lat} +lon_0={_base_lon} +h_0={_base_alt} "
        )

        self._tf_llh2enu = Transformer.from_pipeline(enu_pipeline)

        # Parameters for the NavigateThroughPoses task
        self.max_window    = 50.0   # meters (square side length)
        self.safety_margin = 1.0    # meters (keep a little inside the edge)
        self.max_seg_len   = 100.0  # meters of path length per sub-goal
        self.odom_timeout_s = 2.0   # wait this long for first /fixposition msg
        self.max_retries = 1        # re-attempt each failed segment once
        self._last_goal_uuid: UUID | None = None
        self._retries_left     = 1
        
        # Sliding-window parameters
        self.seg_len_max      = 6.0    # m – length of window Nav2 sees
        self.advance_tol      = 1.60   # m – when to roll the window forward
        self.overlap_tol      = 0.30   # m – keep poses this close as overlap
        self._fp_client = None
        
        # Statistics
        self._total_wps   = 0  # filled in start_ntp()
        self._visited_wps = 0  # will be advanced per segment
        
        if MARKER_VIZ:
            latched_qos = QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

            # Path for visualizing waypoints
            self.waypoint_path_pub = self.navigator.create_publisher(
                    Path, "/gps_waypoints_path", latched_qos)

            # MarkerArray for visualizing waypoints
            self.waypoint_marker_pub = self.navigator.create_publisher(
                    MarkerArray, "/gps_waypoints_markers", latched_qos)
        
            # Path for visualizing the full trajectory    
            self.full_traj_pub   = self.navigator.create_publisher(
                Path, "/gps_full_traj", latched_qos)
            self.full_traj_mrk_pub = self.navigator.create_publisher(
                MarkerArray, "/gps_full_traj_markers", latched_qos)
            
    def _get_datum(self, topic: str = "/fixposition/datum",
               timeout_s: float = 3.0) -> tuple[float, float, float]:
        """
        Block until a NavSatFix is received on *topic* (max *timeout_s*).
        Returns (lat [deg], lon [deg], alt [m]).  Raises RuntimeError on timeout.
        """
        datum: list[float] | None = None

        def _cb(msg: NavSatFix):
            nonlocal datum
            datum = [msg.latitude, msg.longitude, msg.altitude]

        sub = self.navigator.create_subscription(
            NavSatFix, topic, _cb, qos_profile_sensor_data)

        deadline = self.navigator.get_clock().now() + rclpy.duration.Duration(
            seconds=timeout_s)

        while rclpy.ok() and datum is None:
            rclpy.spin_once(self.navigator, timeout_sec=0.05)
            if self.navigator.get_clock().now() > deadline:
                raise RuntimeError(f"No NavSatFix on {topic} within {timeout_s}s")

        self.navigator.destroy_subscription(sub)   # one-shot: free resources
        return tuple(datum)

    # Convert (lat, lon, yaw) to PoseStamped in the map frame
    def _latlon_to_pose(self, lat: float, lon: float, yaw: float) -> PoseStamped:
        x, y, _ = self._tf_llh2enu.transform(lat, lon, 0.0)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp    = rclpy.time.Time().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw       = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def start_ntp(self):
        self.navigator.waitUntilNav2Active(localizer="robot_localization")

        # ------------------------------------------------------------
        # 1)  Convert the whole YAML path once
        # ------------------------------------------------------------
        all_poses = [
            self._latlon_to_pose(wp["latitude"], wp["longitude"],
                                self.wp_parser._reverse_yaw(wp["yaw"]) if self.reverse else wp["yaw"])
            for wp in (self.wp_parser.wps_dict["waypoints"][::-1] if self.reverse else
                    self.wp_parser.wps_dict["waypoints"])
        ]
        
        # Publish the full trajectory for visualization
        self._publish_full_traj(all_poses)

        self._total_wps = len(all_poses)
        total_len       = self._path_length(all_poses)
        self.navigator.get_logger().info(
            f"Preparing segments for trajectory of {total_len:0.2f} m "
            f"consisting of {self._total_wps} waypoints…")
        remaining = all_poses                  # “to-do” list
        seg_idx   = 0

        # ------------------------------------------------------------
        # 2)  Prime the first window
        # ------------------------------------------------------------
        robot = self._get_robot_pose()
        window = self._make_window(robot, remaining)        
        new_wps = len(window) - 1              # skip robot pose
        
        self._log_segment(window,
                          overlap_cnt=0,
                          seg_idx    =seg_idx,
                          new_wps    =new_wps)
        self._publish_waypoints(window, seg_idx)
        self.navigator.goThroughPoses(window)
        
        self._visited_wps += new_wps           # update tally *after* queueing
        remaining = remaining[new_wps:]
        seg_idx += 1

        # ------------------------------------------------------------
        # 3)  Continuous streaming loop
        # ------------------------------------------------------------
        while rclpy.ok():
            rclpy.spin_once(self.navigator, timeout_sec=0.05)

            if self.navigator.isTaskComplete():
                status = self.navigator.getResult()                 # TaskResult enum

                goal_handle = _get_goal_handle(self.navigator)      # may be None
                current_id  = _get_current_uuid(self.navigator)

                #–— If we can’t identify the result reliably, treat it as current –—
                if goal_handle is None or current_id is None:
                    result_is_current = True
                else:
                    result_id = UUID(bytes=goal_handle.goal_id.uuid)
                    result_is_current = (result_id == UUID(bytes=current_id))

                #-----------------------------------------------------------
                # 1)   *Stale* result from the goal we just pre-empted
                #-----------------------------------------------------------
                if not result_is_current:
                    continue        # ignore and keep streaming

                #-----------------------------------------------------------
                # 2)   Current goal SUCCEEDED
                #-----------------------------------------------------------
                if status == TaskResult.SUCCEEDED:
                    if not remaining:                               # trajectory finished
                        self.navigator.get_logger().info("Trajectory finished")
                        break
                    else:
                        continue                                    # hand-off to next window

                #-----------------------------------------------------------
                # 3)   Current goal FAILED  → one retry
                #-----------------------------------------------------------
                if status == TaskResult.FAILED:
                    if self._retries_left > 0:
                        self._retries_left -= 1

                        robot = self._get_robot_pose()

                        # 1.  Walk forward inside the failed window until you
                        #     reach the first pose that is *ahead* of the robot.
                        remaining_in_window = []
                        robot_found_tail = False
                        for p in window:
                            if robot_found_tail or self._dist(robot, p) > 1e-3:
                                robot_found_tail = True
                                remaining_in_window.append(p)

                        # 2.  If nothing left → give up (unlikely but safe-guard)
                        if not remaining_in_window:
                            self.navigator.get_logger().error(
                                "Retry requested but no poses remain in failed window")
                            break

                        # 3.  New window = [robot_pose] + the untouched tail
                        retry_window = [robot] + remaining_in_window

                        self.navigator.get_logger().warn(
                            f"Segment {seg_idx+1} failed – retrying {len(remaining_in_window)} poses")
                        self._publish_waypoints(retry_window, seg_idx)  # same seg-idx: overwrite colour
                        self.navigator.goThroughPoses(retry_window)
                        continue      # <-- DO NOT modify `remaining` or `_visited_wps` here
                    else:
                        self.navigator.get_logger().error("NavigateThroughPoses failed twice – aborting")
                        break

            # Live pose
            robot = self._get_robot_pose()

            # Are we close to the window’s tail?
            if self._dist(robot, window[-1]) > self.advance_tol:
                continue   # still driving inside current window
            
            # Slide the window forward
            tail = window[-1]                      # end-pose of current window

            # -------- overlap: walk *backwards* inside the old window --------
            dist_rt = self._dist(robot, tail)        # robot → tail

            overlap = []
            for p in reversed(window):
                if (self._dist(p, tail) <= self.overlap_tol and      # close to tail
                    self._dist(robot, p) <= dist_rt + 1e-6):         # not behind robot
                    overlap.append(p)
                else:
                    break
            overlap.reverse()

            if not remaining:
                continue   # path finished; let Nav2 coast to the end

            # -------- build the forward part of the new window ---------------
            forward  = self._make_window(tail, remaining)[1:]  # skip dup ‘tail’
            new_window = overlap + forward

            # drop from ‘remaining’ exactly the poses we just attached
            remaining = remaining[len(forward):]
            
            # log stats **before** we cancel / send
            self._log_segment(new_window,
                              overlap_cnt=len(overlap),
                              seg_idx    =seg_idx,
                              new_wps    =len(forward))

            # # Pre-empt the running action
            # self.navigator.cancelTask()
            # # ─── Wait until the cancellation is fully processed ───
            # cancel_deadline = self.navigator.get_clock().now() + rclpy.duration.Duration(
            #     seconds=3.0)                        # plenty – normally < 0.3 s
            # while not self.navigator.isTaskComplete():
            #     if self.navigator.get_clock().now() > cancel_deadline:
            #         self.navigator.get_logger().error("Cancel timeout – aborting")
            #         return
            #     rclpy.spin_once(self.navigator, timeout_sec=0.05)
                
            # # ─────────────────────────────────────────────────────────────
            # #  Extra guard: wait until internal follow_path is cancelled
            # # ─────────────────────────────────────────────────────────────
            # if self._fp_client is None:
            #     self._fp_client = ActionClient(self.navigator, FollowPath, "follow_path")
            #     if not self._fp_client.wait_for_server(timeout_sec=2.0):
            #         self.navigator.get_logger().warn("follow_path server not available")

            # fp_deadline = self.navigator.get_clock().now() + rclpy.duration.Duration(
            #     seconds=3.0)
            # while True:
            #     goal_handles = self._fp_client._goal_handles  # internal list
            #     if not goal_handles:
            #         break                                   # nothing active
            #     status = goal_handles[0].status
            #     if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED):
            #         break
            #     if self.navigator.get_clock().now() > fp_deadline:
            #         self.navigator.get_logger().error("follow_path cancel timeout")
            #         return
            #     rclpy.spin_once(self.navigator, timeout_sec=0.05)
            
            self._publish_waypoints(new_window, seg_idx)
            self.navigator.goThroughPoses(new_window)
            self._visited_wps += len(forward)   # advance tally
            seg_idx += 1
            window = new_window
        
    def _odom_cb(self, msg: Odometry) -> None:
        """Cache the most recent odometry pose as a `PoseStamped`."""
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose   = msg.pose.pose
        self._last_odom_pose = ps

    def _get_robot_pose(self) -> PoseStamped:
        """
        Return the most recent pose from /fixposition/odometry_enu.
        Blocks (spins) until a message arrives or the timeout elapses.
        """
        deadline = self.navigator.get_clock().now() + rclpy.duration.Duration(
            seconds=self.odom_timeout_s)

        while self._last_odom_pose is None:
            if self.navigator.get_clock().now() > deadline:
                raise RuntimeError(
                    f"No Odometry on /fixposition/odometry_enu "
                    f"after {self.odom_timeout_s:.1f}s")
            rclpy.spin_once(self.navigator, timeout_sec=0.05)

        return self._last_odom_pose
        
    def _publish_waypoints(self, poses, seg_idx: int):
        # Path message
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp    = rclpy.time.Time().to_msg()
        path.poses           = poses
        
        # RViz colours Paths per-topic, not per-message → we publish a Marker
        # LINE_STRIP instead (same header) so each segment gets its own hue.
        line = Marker()
        line.header = path.header
        line.ns     = "seg_line"
        line.id     = seg_idx          # unique id per segment
        line.type   = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0  # identity
        line.scale.x = 0.03            # line width
        hue = (seg_idx * 0.18) % 1.0   # golden-ratio hop around the circle
        r, g, b = _hsv_to_rgb(hue, 1.0, 1.0)
        line.color.r, line.color.g, line.color.b, line.color.a = r, g, b, 1.0
        # copy points
        for p in poses:
            line.points.append(p.pose.position)

        self.waypoint_path_pub.publish(path)   # still publish for tooling
        self.waypoint_marker_pub.publish(MarkerArray(markers=[line]))

        # Optional coloured spheres
        markers = MarkerArray()
        for i, p in enumerate(poses):
            m = Marker()
            m.header   = path.header
            m.ns       = "wps"
            m.id       = seg_idx*1000 + i
            m.type     = Marker.SPHERE
            m.action   = Marker.ADD
            m.pose     = p.pose
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            markers.markers.append(m)
        self.waypoint_marker_pub.publish(markers)
        
    def _dist(self, a: PoseStamped, b: PoseStamped) -> float:
        dx, dy = a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y
        return math.hypot(dx, dy)
    
    def _path_length(self, poses: list[PoseStamped]) -> float:
        return sum(self._dist(a, b) for a, b in zip(poses[:-1], poses[1:]))

    def _make_window(self, start: PoseStamped, todo: list[PoseStamped]) -> list[PoseStamped]:
        """Return [start] + as many todo-poses as fit within seg_len_max."""
        window  = [start]
        length  = 0.0
        prev_xy = (start.pose.position.x, start.pose.position.y)

        for p in todo:
            step = math.hypot(p.pose.position.x - prev_xy[0],
                            p.pose.position.y - prev_xy[1])
            if length + step > self.seg_len_max:
                break
            window.append(p)
            length, prev_xy = length + step, (p.pose.position.x, p.pose.position.y)
        return window
    
    def _segment_length(self, poses: list[PoseStamped]) -> float:
        return sum(self._dist(a, b) for a, b in zip(poses[:-1], poses[1:]))

    def _log_segment(self, window,
                     *, overlap_cnt: int,
                     seg_idx: int,
                     new_wps: int) -> None:
        seg_len  = self._segment_length(window)
        num_wps  = len(window)
        progress = max(self._visited_wps - overlap_cnt, 0)

        self.navigator.get_logger().info(
            f"Segment {seg_idx+1} length: {seg_len:0.2f} m, "
            f"Num. waypoints: {num_wps}, "
            f"Overlap: {overlap_cnt}, "
            f"Progress: {progress}/{self._total_wps}")
        
    def _publish_full_traj(self, poses: list[PoseStamped]):
        """Publish the whole trajectory as a latched Path + MarkerArray."""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp    = rclpy.time.Time().to_msg()
        path.poses           = poses
        self.full_traj_pub.publish(path)

        # one violet line-strip + tiny white spheres at every waypoint
        line = Marker()
        line.header = path.header
        line.ns     = "full_traj"
        line.id     = 0
        line.type   = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.04
        line.color.r, line.color.g, line.color.b, line.color.a = 0.7, 0.3, 1.0, 1.0
        line.points = [p.pose.position for p in poses]

        dots = MarkerArray()
        for i, p in enumerate(poses):
            m = Marker()
            m.header = path.header
            m.ns, m.id = "full_traj_pts", i
            m.type, m.action = Marker.SPHERE, Marker.ADD
            m.pose = p.pose
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r = m.color.g = m.color.b = 1.0
            m.color.a = 1.0
            dots.markers.append(m)

        self.full_traj_mrk_pub.publish(MarkerArray(
            markers=[line] + dots.markers))


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
