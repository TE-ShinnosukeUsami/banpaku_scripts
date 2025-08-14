#!/usr/bin/env bash

SESSION="banpaku_demo_session"

tmux new-session -d -s "${SESSION}"

# Window 0
tmux rename-window -t "${SESSION}:0" "sensor"
tmux send-keys -t "${SESSION}:0" ". ~/workspace/banpaku_scripts/jetson/00_start_sensors.sh"

# Window 1
tmux rename-window -t "${SESSION}:1" "exposure"
tmux send-keys -t "${SESSION}:1" ". ~/workspace/banpaku_scripts/jetson/01_set_exposure_and_remove_docker_container.sh"

tmux attach-session -t "${SESSION}"
