# Fixposition VRTK2 Nav2 Tutorial: Waypoint Follower

# Setup Tutorial

## Step 1: Set up workspace
### Environment
1. Ubuntu 24.04 LTS
2. ROS Version: ROS2 Jazzy

### ROS Packages
The following ROS packages are required for this project:

1. Scout Driver ([scout_ros2](https://github.com/westonrobot/scout_ros2.git), [ugv_sdk](https://github.com/westonrobot/ugv_sdk.git))
2. Fixposition Vision-RTK2 Driver ([fixposition_driver](https://github.com/fixposition/fixposition_driver.git))
3. Fixposition Nav2 Tutorial ([nav2_tutorial](https://github.com/fixposition/nav2_tutorial.git))

To obtain them, simply execute the following command at the root of the repository:
```bash
git submodule update --init --recursive
```


### (Optional) Docker container
The user can also compile the provided Docker container in the .devcontainer folder to test this tutorial. To achieve this, the following commands can be used:
```bash
docker compose -f .devcontainer/docker-compose.yaml build
docker compose -f .devcontainer/docker-compose.yaml up -d
docker compose -f ~/dev/nav2_tutorial/.devcontainer/docker-compose.yaml exec vrtk bash
```
Alternatively, the user can compile it directly using the Dev Containers extension in VSCode.

To check the status of the different services, you can run the following commands:
```bash
docker compose logs vrtk
```


## Step 2: Set up Fixposition ROS Driver

To use the ROS driver with the Scout robot, the following changes must be applied:

1. Enable the following messages in the I/O configuration page of the sensor:
    - FP_A-ODOMETRY
    - FP_A-ODOMENU
    - FP_A-ODOMSH
    - FP_A-LLH
    - FP_A-EOE_FUSION
    - FP_A-TF_VRTKCAM	
    - FP_A-TF_POIVRTK	
    - FP_A-TF_ECEFENU0 	
    - FP_A-TF_POIPOISH 	
    - FP_A-TF_POIIMUH

2. In the configuration file of the ROS driver (src/fixposition_driver/fixposition_driver_ros2/launch/config.yaml), change the 'nav2_mode' to 'true' and 'qos_type' to 'default_long'.

3. Run the setup_ros_ws bash script to set up the fixposition driver accordingly.

4. (Optional) Configure wheelspeed measurements by setting up the following configuration:
```YAML
converter:
    enabled: true
    topic_type: "Odometry"     # Supported types: nav_msgs/{Twist, TwistWithCov, Odometry}
    input_topic: "/odom"       # Input topic name
    scale_factor: 1000.0       # To convert from the original unit of measurement to mm/s (note: this must be a float!)
    use_x: true                # Transmit the x axis of the input velocity
    use_y: false               # Transmit the y axis of the input velocity
    use_z: false               # Transmit the z axis of the input velocity
```


## Step 3: Build ROS2 workspace
Build the ROS2 workspace.
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```


## Step 4: Source built packages
```bash
source install/setup.bash
```


## Step 5: Test Scout Driver

### Establish connection with the Scout robot
To communicate with the Scout robot, you must use the provided USB-to-CAN adapter. Then, run the following commands to start the connection:

```bash
sudo modprobe gs_usb can-utils
sudo ip link set can0 up type can bitrate 500000
candump can0
```

Alternatively, the user can also directly execute the provided script setup-can.sh:
```bash
sudo ./scripts/setup-can.sh
```

Example output from the can0 port:
```bash
can0 311 [8] 00 00 25 C6 FF FF F0 A4
can0 251 [8] 00 00 00 00 00 00 00 00
can0 252 [8] 00 00 00 00 00 00 00 00
can0 253 [8] 00 00 00 00 00 00 00 00
can0 254 [8] 00 00 00 00 00 00 00 00
can0 241 [8] AA 00 00 00 00 00 00 ED
can0 221 [8] 00 00 00 00 00 00 00 00
can0 311 [8] 00 00 25 C6 FF FF F0 A4
...
```

Note: To automatically establish the connection to the Agilex, you can edit the /etc/modules file as such:
```bash
# /etc/modules: kernel modules to load at boot time.
gs_usb
can-utils
```

In addition, in case the error messsage 'RTNETLINK answers: Device or resource busy' appears, please run the following command:
```bash
sudo ip link set can0 down
```

### Control the Scout robot using the keyboard

In terminal 1, run these commands to start the base node for the Scout robot:
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch scout_base scout_mini_base.launch.py
```

In terminal 2, run these commands to start the keyboard-based controller:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# 🧭 GPS Waypoint Loggers for ROS 2

This package provides two ROS 2 nodes for recording GPS waypoints to a YAML file, based on GNSS and odometry data:

| Script                   | Logging Mode         | Trigger        | Ideal Use Case                  |
| ------------------------ | -------------------- | -------------- | ------------------------------- |
| `gps_keylogger.py`       | Manual               | Keyboard `'f'` | Sparse or event-based recording |
| `gps_periodic_logger.py` | Automated (periodic) | Time interval  | Continuous route logging        |

---

## 🧰 Topics Subscribed

Both scripts subscribe to:

* `/fixposition/odometry_llh` — `sensor_msgs/NavSatFix`
* `/fixposition/odometry_enu` — `nav_msgs/Odometry`

---

## 🗺 Waypoint Format

Logged waypoints are stored in a YAML file under the `waypoints:` key. Each entry includes:

```yaml
waypoints:
  - latitude: 47.123456
    longitude: 8.654321
    altitude: 520.4
    yaw: 1.57
    logged_at: "2025-06-30T15:47:12.003918"
```

---

## 🚀 How to Use

### Manual Logger (`gps_keylogger.py`)

```bash
ros2 run nav2_tutorial gps_keylogger [optional_output.yaml]
```

* Press `'f'` to log a waypoint
* Press `'q'` to quit and save

Waypoints are saved immediately and safely to disk.

---

### Periodic Logger (`gps_periodic_logger.py`)

```bash
ros2 run nav2_tutorial gps_periodic_logger [optional_output.yaml] -i 0.2
```

* Logs waypoints automatically every 0.2s (default)
* Press `'q'` to stop recording
* Uses buffered writing to reduce I/O overhead
* Filters out insignificant movement (less than 1 cm).

---

## 📦 Output Location

If no output path is specified, the log will be written to:
```
nav2_tutorial/src/nav2_tutorial/trajectories/gps_waypoints_<timestamp>.yaml
```

## 📈 Visualizing Logged Trajectories

To quickly visualize GPS waypoint logs, use the `visualize_gps_yaml.py` script:

### Example:
```bash
python3 utils/visualize_gps_yaml.py path/to/gps_waypoints.yaml         # simple 2D plot
python3 utils/visualize_gps_yaml.py path/to/gps_waypoints.yaml --map   # map overlay (if supported)
```

### Requirements for Map Overlay:
- `geopandas`
- `contextily`
- `shapely`

This will display your trajectory as a line over OpenStreetMap tiles (Mapnik style) using Web Mercator projection.



# GPS Waypoint Follower
Launch the ROS nodes in the following order:
```bash
ros2 launch scout_base scout_mini_base.launch.py
ros2 launch nav2_tutorial fp_driver_node.launch config:=fp_driver_config.yaml
ros2 launch nav2_tutorial gps_waypoint_follower.launch.py
```
Note: You can use the `all_nodes.launch.py` file to achieve this. For navigation, this repository provides three waypoint following methods: Precise, Smooth, and Interactive. We can only choose one method each time we execute it.

* Precise GPS Waypoint Follower
The `precise_wp_follower` script streams all the logged points from a YAML file and makes the robot follow them point-by-point, stopping at every waypoint when the accuracy threshold is met.
```bash
ros2 run nav2_tutorial precise_wp_follower <optional: /path/to/file> <optional: --last> <optional: --reverse>
```

* Smooth GPS Waypoint Follower
The `smooth_wp_follower` script streams all the logged points from a YAML file and divides them in segments, pre-computing a smooth trajectory for the robot to follow. This ensure constant speed throughout the waypoint following task.
```bash
ros2 run nav2_tutorial smooth_wp_follower <optional: /path/to/file> <optional: --last> <optional: --reverse>
```

* Interactive GPS Waypoint Follower
The `interactive_wp_follower` script listens to the mapviz topic for the next waypoint objective.
```bash
ros2 run nav2_tutorial interactive_wp_follower
```

For launching the graphical interface, you can run the following command (note that this only works on x86/amd64 devices):
```bash
ros2 launch nav2_tutorial mapviz.launch.py
```
Or alternatively use the `--mapviz` flag on the `gps_waypoint_follower.launch.py` script.


# Debugging trajectories
To analyze the trajectories performed by the robot and determine if any issues have occured, the user can perform a recording of all topics by running `ros2 bag record --all`. Then, launch RViz2 with the provided configuration file to easily visualize the debug topics containing the waypoint coordinates, the global and local plans, the costmaps, among other things: `ros2 run rviz2 rviz2 -d /home/dev/ros_ws/src/nav2_tutorial/config/nav2_plan_viz.rviz`.

# Recommendations
We suggest employing tmux for controlling the robot via SSH, as intermittent disconnections may kill the running process. Next you'll find some basic steps to get familiar with tmux:

### Installing tmux
To install tmux on your device, please run the following commands:
```bash
sudo apt update && apt install tmux
```

### Start a tmux session
After you SSH into the device (using a terminal or VSCode's Remote-SSH), start a new tmux session as:
```bash
tmux new -s mysession
```
where "mysession" is simply the name given to this tmux session. Note that the prompt will change slightly once inside the tmux session. Inside, you can interact with it as a normal terminal and run any process.

### Detach from tmux
At any point, the user can "detach" from tmux and the process will keep running in the background. To do this, please press: 'Ctrl + b' (release both) then press 'd'. This detaches from the session and brings you back to the normal shell while the program continues running.

### Reconnect later
If the SSH connection drops or you log in again later, list running tmux sessions as:
```bash
tmux ls
```

The user will be prompted with something like:
```bash
mysession: 1 windows (created Thu Jul  3 14:32:51 2025) [80x24]
```

To reattach to the tmux session, simply use:
```bash
tmux attach -t mysession
```
You’re now back inside the process, exactly where you left off.
