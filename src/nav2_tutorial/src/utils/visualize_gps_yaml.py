#!/usr/bin/env python3
import argparse
import yaml
import matplotlib.pyplot as plt
import contextlib

# Optional map overlay
with contextlib.suppress(ImportError):
    import contextily as ctx


def load_waypoints(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('waypoints', [])


def plot_waypoints(waypoints, use_map=False):
    if not waypoints:
        print("No waypoints found.")
        return

    lats = [wp['latitude'] for wp in waypoints]
    lons = [wp['longitude'] for wp in waypoints]

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(lons, lats, marker='o', linestyle='-', color='blue', label='Trajectory')
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("GPS Waypoint Trajectory")
    ax.axis('equal')
    ax.grid(True)
    ax.legend()

    if use_map:
        try:
            import geopandas as gpd
            from shapely.geometry import Point

            # Convert to GeoDataFrame and plot with contextily
            gdf = gpd.GeoDataFrame(
                geometry=[Point(lon, lat) for lon, lat in zip(lons, lats)],
                crs="EPSG:4326"
            ).to_crs(epsg=3857)

            gdf.plot(ax=ax, marker='o', color='red')
            ctx.add_basemap(ax, source=ctx.providers.OpenStreetMap.Mapnik)
        except ImportError:
            print("Map overlay requires 'geopandas' and 'contextily'. Falling back to plain plot.")

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Visualize GPS waypoints from YAML")
    parser.add_argument("yaml_file", help="Path to the YAML file containing waypoints")
    parser.add_argument("--map", action="store_true", help="Overlay trajectory on a map (requires geopandas + contextily)")
    args = parser.parse_args()

    waypoints = load_waypoints(args.yaml_file)
    plot_waypoints(waypoints, use_map=args.map)


if __name__ == "__main__":
    main()
