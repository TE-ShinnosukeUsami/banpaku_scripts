#!/bin/bash

SCRIPT_DIRECTORY="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)"

# Fast-RTPS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Reset Discovery Server v2 settings
echo "unset Discovery Server v2 and reset Simple Discovery."
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIRECTORY}/fastdds_profile.xml

echo "Restart ROS2 CLI to use Simple Discovery configuration."
ros2 daemon stop
ros2 daemon start
