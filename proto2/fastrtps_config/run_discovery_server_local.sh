#!/bin/bash

SCRIPT_DIRECTORY="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)"

# Warning
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "Warning: This script must be sourced. Use '. ./hoge.sh' or 'source hoge.sh' instead of './hoge.sh'."
    exit 1
fi

# Fast-RTPS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Super Client for ROS2 CLI
echo "Setting ROS2 CLI to use Discovery SUPER_CLIENT configuration."
export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIRECTORY}/discovery_super_client.xml
ros2 daemon stop
ros2 daemon start

# Discovery Server
echo "Run Default Discovery Server."
fastdds discovery -i 0 -x ${SCRIPT_DIRECTORY}/discovery_server_local.xml
