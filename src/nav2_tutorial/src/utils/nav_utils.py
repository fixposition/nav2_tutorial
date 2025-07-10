import math
from typing import List
from geometry_msgs.msg import PoseStamped


def _pose_dist(a: PoseStamped, b: PoseStamped) -> float:
    """Return the Euclidean distance between two PoseStamped messages."""
    dx, dy = a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y
    return math.hypot(dx, dy)


def _path_length(poses: list[PoseStamped]) -> float:
    """Return the total length of the path defined by the poses."""
    return sum(_pose_dist(a, b) for a, b in zip(poses[:-1], poses[1:]))


def _segment_length(poses: list[PoseStamped]) -> float:
    """Return the length of the segment defined by the poses."""
    return sum(_pose_dist(a, b) for a, b in zip(poses[:-1], poses[1:]))


def _closest_wp_index(robot: PoseStamped, poses: list[PoseStamped]) -> int:
    """Compute the index of the closest waypoint to the robot."""
    d_min, idx_min = float("inf"), 0
    for i, p in enumerate(poses):
        d = _pose_dist(robot, p)
        if d < d_min:
            d_min, idx_min = d, i
    return idx_min


def _make_window(start: PoseStamped, todo: list[PoseStamped], seg_len_max: float) -> list[PoseStamped]:
    """Return [start] + as many todo-poses as fit within seg_len_max."""
    window  = [start]
    length  = 0.0
    prev_xy = (start.pose.position.x, start.pose.position.y)

    for p in todo:
        step = math.hypot(p.pose.position.x - prev_xy[0],
                        p.pose.position.y - prev_xy[1])
        if length + step > seg_len_max:
            break
        window.append(p)
        length, prev_xy = length + step, (p.pose.position.x, p.pose.position.y)
    return window


def _hsv_to_rgb(h: float, s: float = 1.0, v: float = 1.0):
    """HSV to RGB converter."""
    i = int(h * 6.0) % 6
    f = h * 6.0 - i
    p, q, t = v * (1 - s), v * (1 - f * s), v * (1 - (1 - f) * s)
    if   i == 0: r, g, b = v, t, p
    elif i == 1: r, g, b = q, v, p
    elif i == 2: r, g, b = p, v, t
    elif i == 3: r, g, b = p, q, v
    elif i == 4: r, g, b = t, p, v
    else:        r, g, b = v, p, q
    return r, g, b


def _get_goal_handle(nav):
    """Return the ActionGoal handle of the latest NavigateThroughPoses goal."""
    for attr in ("_ntp_goal_handle",               # Iron, Jazzy
                 "_nav_through_poses_goal_handle"  # Humble, Galactic
                ):
        if hasattr(nav, attr):
            return getattr(nav, attr)
    return None

def _get_current_uuid(nav):
    """Return the UUID (bytes) of the goal that is currently active."""
    for attr in ("_current_ntp_uuid"                 # Iron/Jazzy
                 "_current_nav_through_poses_uuid",  # Humble, Galactic        
                ):
        if hasattr(nav, attr):
            return getattr(nav, attr)
    return None