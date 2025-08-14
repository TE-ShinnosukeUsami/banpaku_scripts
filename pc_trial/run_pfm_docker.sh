#!/bin/bash

# コンテナ名
CONTAINER_NAME="ros-humble"

# コンテナの存在確認
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "コンテナ '${CONTAINER_NAME}' が存在します。docker exec で入ります。"
    docker exec -it "${CONTAINER_NAME}" bash
else
    echo "コンテナ '${CONTAINER_NAME}' が存在しません。launch.sh を実行します。"
    cd ~/workspace/SR-docker_env/ROS_HUMBLE_VDETR || exit 1
    ./launch.sh ~/workspace/PFM/ros2_ws/ ~/workspace/PFM/workspace/
fi
