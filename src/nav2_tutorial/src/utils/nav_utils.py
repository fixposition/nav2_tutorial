def _hsv_to_rgb(h, s=1.0, v=1.0):
    """HSV to RGB converter."""
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
    for attr in ("_current_nav_through_poses_uuid",  # Humble
                 "_current_ntp_uuid"                 # Iron/Jazzy
                ):
        if hasattr(nav, attr):
            return getattr(nav, attr)
    return None