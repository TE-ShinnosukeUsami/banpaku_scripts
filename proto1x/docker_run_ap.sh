#!/bin/bash

# Discovery Client
. ~/workspace/aii-proto/jetson/ros2_ws/fastrtps_config/set_discovery_client.sh

# MP Reset


. ~/workspace/riza/ros2_ws/install/setup.bash
ros2 launch aii_proto_bringup robohome3d.launch.py isaac_sim:=false robot_type:=PROTO1X
