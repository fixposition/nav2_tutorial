# AgileX Scout Mini GPS Waypoint Follower Setup

## Environment
1. Ubuntu 22.04 LTS
2. ROS Version: ROS2 Humble

## ROS Packages
1. Scout Driver (scout_ros2, ugv_sdk)
2. Fixposition Vision-RTK2 Driver (fixposition_driver)
3. AgileX Nav2 Demo (nav2_tutorial)

# Setup Tutorial
### Step 1: Build ROS Packages Above
Create a ROS2 workspace.
```
mkdir -p ~/ros2_humble_ws/src
source /opt/ros/humble/setup.bash
```

Clone relevant ROS packages and make some changes.
```
cd ~/ros2_humble_ws/src

# Scout Driver
git clone https://github.com/westonrobot/ugv_sdk.git
git clone https://github.com/westonrobot/scout_ros2.git
cd scout_ros2
git checkout humble

sudo gedit ~/ros2_humble_ws/src/scout_ros2/scout_base/include/scout_base/scout_messenger.hpp
## Comment out line 263 and save. (tf_broadcaster_->sendTransform(tf_msg);)
cd ..

# Fixposition Vision-RTK2 Driver
git clone https://github.com/fixposition/fixposition_driver.git

# AgileX Nav2 Demo
git clone https://github.com/fixposition/nav2_tutorial.git

# Build these ros packages
cd ~/ros2_humble_ws
colcon build
source install/setup.bash
```

### Step 2: Test Scout Driver
Use USB-to-CAN adapter to connect to AgileX first.

```
sudo modprobe gs_usb can-utils
sudo ip link set can0 up type can bitrate 500000
candump can0
# You can see message from CAN0 like below.
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

Use keyboard to control AgileX Scout Mini.

Terminal 1:
```
source /opt/ros/humble/setup.bash && source install/setup.bash
# start the base node for the scout mini robot
ros2 launch scout_base scout_mini_base.launch.py
```

Terminal 2:
```
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Step 3: Test Fixposition Vision-RTK2 Driver
Initialize Vision-RTK2 and make sure all converged.

```
# Need the following two output
FP_A-ODOMETRY (/fixposition/ypr)
FP_A-LLH (/fixposition/navsatfix)
```

### Step 4: Start GPS Waypoint Following
First, make sure that the following things have been completed.
1. Launch Scout Mini.
2. Vision-RTK2 fusion works properly and all converge.
3. Computer connects to AgileX Scout Mini.

Then, launch nodes in the following order.
```
ros2 launch scout_base scout_mini_base.launch.py
ros2 launch fixposition_driver_ros2 tcp.launch
ros2 launch nav2_tutorial gps_waypoint_follower.launch.py use_mapviz:=True
```

Finally, start navigation. There are two types waypoint following methods. We can only choose one method each time we execute it.

* Interactive GPS Waypoint Follower
```
ros2 run nav2_tutorial interactive_waypoint_follower
```


* Logged GPS Waypoint Follower

First, we must get a predefined gps_waypoints.yaml file that we want the robot to follow. One method is that we give GPS points manually. Another is that we can use waypoint logging tool to get this file just like below.
```
ros2 run nav2_tutorial gps_waypoint_logger
```

Second, We can use logged_waypoint_follower to make the robot follow the logged waypoints.
```
ros2 run nav2_tutorial logged_waypoint_follower ~/gps_waypoints.yaml
```
