#!/usr/bin/env bash
set -euo pipefail

# ==============================
USER_PASS="sony1234"
REMOTE_DISCOVERY_SERVER_ID="1"
# ==============================

# ANSI escape sequence definitions
ESC=$(printf '\033')
CSI="${ESC}["
RESET="${CSI}0m"
GREEN="${CSI}32m"
ERASE_LINE="${CSI}2K"
HIDE_CURSOR="${CSI}?25l"
SHOW_CURSOR="${CSI}?25h"

# Spinner function
function spinner() {
  local i=0
  local spin='⠧⠏⠛⠹⠼⠶'
  local n=${#spin}
  while true; do
    sleep 0.1
    printf "%s" "${ERASE_LINE}"
    printf "%s %s" "${GREEN}${spin:i++%n:1}${RESET}" "$*"
    printf "\r%s" "${HIDE_CURSOR}"
  done
}

# Stop spinner helper
function stop_spinner() {
  local pid=$1
  kill "${pid}" >/dev/null 2>&1 || true
  wait "${pid}" 2>/dev/null || true
  printf "%s" "${SHOW_CURSOR}"
}

# Request sudo upfront
echo -n "Requesting sudo credentials... "
sudo -v
echo "Done."
echo

SESSION="discovery_server_and_ap"
# Create session
tmux new-session -d -s "${SESSION}"

# Split window into 3 horizontal sections: top 33%, middle 33%, bottom 34%
tmux split-window -v -t "${SESSION}" -p 66
tmux split-window -v -t "${SESSION}" -p 50

# Now split each of the three panes into two columns
tmux select-pane -t "${SESSION}.0"
tmux split-window -h -t "${SESSION}.0"
tmux select-pane -t "${SESSION}.1"
tmux split-window -h -t "${SESSION}.1"
tmux select-pane -t "${SESSION}.2"
tmux split-window -h -t "${SESSION}.2"

# Finally tile layout to equalize any minor differences
tmux select-layout -t "${SESSION}:0" tiled

echo "Created TMUX session '${SESSION}' with 6 equal panes."

echo

# Examine in Pane 0: launch container and run local discovery server
echo "Pane 0: Launching Docker..."
tmux send-keys -t "${SESSION}:0.0" '. ~/workspace/banpaku_scripts/proto1x/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.0" ${USER_PASS} C-m

# wait for 'local_discovery' container
echo "Pane 0: Waiting for 'local_discovery' container to start..."
(
  spinner "Checking container status..."
)&
SPINNER_PID=$!
until docker container ls --format '{{.Names}}' | grep -q jetson_aarch64; do sleep 0.5; done
stop_spinner "${SPINNER_PID}"
sleep 1

echo "Pane 0: Local discovery is running."
tmux send-keys -t "${SESSION}:0.0" ". ~/workspace/banpaku_scripts/proto1x/docker_run_local_discovery_server.sh" C-m

# Examine in Pane 2: run remote discovery server
echo "Pane 2: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.2" '. ~/workspace/banpaku_scripts/proto1x/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.2" ${USER_PASS} C-m
sleep 1
echo "Pane 2: Starting remote discovery server..."
tmux send-keys -t "${SESSION}:0.2" '. ~/workspace/banpaku_scripts/proto1x/docker_run_remote_discovery_server.sh' C-m
tmux send-keys -t "${SESSION}:0.2" '0' C-m
tmux send-keys -t "${SESSION}:0.2" ${REMOTE_DISCOVERY_SERVER_ID} C-m

# Examine in Pane 1: run AP container
echo "Pane 1: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.1" '. ~/workspace/banpaku_scripts/proto1x/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.1" ${USER_PASS} C-m
sleep 1
echo "Pane 1: Starting AP service..."
tmux send-keys -t "${SESSION}:0.1" '. ~/workspace/banpaku_scripts/proto1x/docker_run_ap.sh' C-m

# Examine in Pane 3: tf2_echo
echo "Pane 3: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.3" '. ~/workspace/banpaku_scripts/proto1x/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.3" ${USER_PASS} C-m
sleep 1
echo "Pane 3: setup SUPER_CLIENT"
tmux send-keys -t "${SESSION}:0.3" '. ~/workspace/banpaku_scripts/proto1x/docker_setup_commander.sh' C-m
sleep 1
echo "Pane 3: Starting tf2_echo for world->base_link ..."
tmux send-keys -t "${SESSION}:0.3" "ros2 run tf2_ros tf2_echo world ${HOSTNAME}/base_link" C-m

# Examine in Pane 4: commander
echo "Pane 4: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.4" '. ~/workspace/banpaku_scripts/proto1x/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.4" ${USER_PASS} C-m
sleep 1
echo "Pane 4: setup SUPER_CLIENT"
tmux send-keys -t "${SESSION}:0.4" '. ~/workspace/banpaku_scripts/proto1x/docker_setup_commander.sh' C-m

# Examine in Pane 5: commander
echo "Pane 5: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.5" '. ~/workspace/banpaku_scripts/proto1x/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.5" ${USER_PASS} C-m
sleep 1
echo "Pane 5: setup SUPER_CLIENT"
tmux send-keys -t "${SESSION}:0.5" '. ~/workspace/banpaku_scripts/proto1x/docker_setup_commander.sh' C-m
tmux send-keys -t "${SESSION}:0.5" 'ros2 node list' C-m

tmux attach-session -t "${SESSION}"

