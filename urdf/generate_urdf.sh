#!/bin/bash

ros2 run xacro xacro scout_v2.xacro > scout_v2.urdf
ros2 run xacro xacro scout_vrtk.xacro > scout_vrtk.urdf
