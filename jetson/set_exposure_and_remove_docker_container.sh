#!/bin/bash

v4l2-ctl -d /dev/video1 --set-ctrl exposure=300
v4l2-ctl -d /dev/video1 -L

docker container kill jetson_aarch64
docker container rm jetson_aarch64
