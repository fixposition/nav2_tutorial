import os
import yaml
import rclpy
import argparse
import numpy as np
import matplotlib.pyplot as plt
from src.utils.parser_utils import YamlWaypointParser, _select_yaml


def _first_waypoint(path):
    with open(path) as f:
        return yaml.safe_load(f)["waypoints"][0]

def load_enu(path, base=None):
    """Return two numpy arrays (E, N) in meters."""
    if base is None:
        wp0 = _first_waypoint(path)
        base = (wp0["latitude"], wp0["longitude"], wp0.get("altitude", 0.0))

    parser = YamlWaypointParser(path, *base)
    poses  = parser.get_wps()
    east   = np.array([p.pose.position.x for p in poses])
    north  = np.array([p.pose.position.y for p in poses])
    return east, north

def main():
    rclpy.init()
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml_file", nargs="?",
                    help="Waypoint file (if omitted, auto-selection applies)")
    ap.add_argument("--last", action="store_true",
                    help="Pick the lexically newest logfile in trajectories/")
    ap.add_argument("--base", type=float, nargs=3, metavar=("LAT","LON","ALT"),
                    help="Explicit ENU base point; defaults to first waypoint")
    args = ap.parse_args()
    
    base_path, _ = os.path.splitext(args.yaml_file)
    output_path = base_path + ".png"

    yaml_path = _select_yaml(args)
    east, north = load_enu(yaml_path, base=args.base)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(east, north, "-o", markersize=3, linewidth=1.2)

    ax.set_title(os.path.basename(yaml_path))
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_aspect("equal")
    ax.grid(True)
    plt.tight_layout()
    plt.savefig(output_path)
    print(f"Saved plot to: {output_path}")

if __name__ == "__main__":
    main()
