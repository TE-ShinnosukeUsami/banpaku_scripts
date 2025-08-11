#!/bin/bash

docker stop $(docker ps -a -q)
docker rm $(docker ps -a -q)
tmux kill-session -t banpaku_demo_session
