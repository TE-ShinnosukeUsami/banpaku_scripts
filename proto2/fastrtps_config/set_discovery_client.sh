#!/bin/bash

SCRIPT_DIRECTORY="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)"

# Fast-RTPS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Super Client for ROS2 CLI
echo "Setting ROS2 CLI to use Discovery SUPER_CLIENT configuration."
export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIRECTORY}/discovery_super_client.xml
ros2 daemon stop
ros2 daemon start

# Client
echo "Setting Fast-RTPS default to use Discovery CLIENT configuration."
export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIRECTORY}/discovery_client.xml
