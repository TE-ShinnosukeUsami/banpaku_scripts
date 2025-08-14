#!/usr/bin/env bash
set -euo pipefail

# ==============================
USER_PASS="sony1234"
REMOTE_DISCOVERY_SERVER_ID="3"
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

SESSION="banpaku_demo_session"
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
# Example in Pane 0: start sensors with spinner
echo "Pane 0: starting sensors..."
tmux send-keys -t "${SESSION}:0.0" '. ~/workspace/banpaku_scripts/proto2/start_sensors.sh' C-m

sleep 2
tmux send-keys -t "${SESSION}:0.0" ${USER_PASS} C-m

echo -n "Waiting for sensor init (15s)... "
(
  spinner "Initializing sensors..."
)&
SPINNER_PID=$!
sleep 15
stop_spinner "${SPINNER_PID}"
echo -e "${GREEN}Pane 0: Sensors initialized.${RESET}"
echo

# Examine in Pane 2: set itof exposure, remove and create container, and run local discovery server
echo "Pane 2: Setting exposure and running local discovery..."
tmux send-keys -t "${SESSION}:0.2" '. ~/workspace/banpaku_scripts/proto2/set_exposure_and_remove_docker_container.sh' C-m
echo "Pane 2: Launching Docker..."
tmux send-keys -t "${SESSION}:0.2" '. ~/workspace/banpaku_scripts/proto2/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.2" ${USER_PASS} C-m

# wait for 'local_discovery' container
echo "Pane 2: Waiting for 'local_discovery' container to start..."
(
  spinner "Checking container status..."
)&
SPINNER_PID=$!
until docker container ls --format '{{.Names}}' | grep -q jetson_aarch64; do sleep 0.5; done
stop_spinner "${SPINNER_PID}"
sleep 1

echo "Pane 2: Local discovery is running."
tmux send-keys -t "${SESSION}:0.2" ". ~/workspace/banpaku_scripts/proto2/docker_run_local_discovery_server.sh" C-m

# Examine in Pane 4: run remote discovery server
echo "Pane 4: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.4" '. ~/workspace/banpaku_scripts/proto2/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.4" ${USER_PASS} C-m
sleep 1
echo "Pane 4: Starting remote discovery server..."
tmux send-keys -t "${SESSION}:0.4" '. ~/workspace/banpaku_scripts/proto2/docker_run_remote_discovery_server.sh' C-m
tmux send-keys -t "${SESSION}:0.4" '0' C-m
tmux send-keys -t "${SESSION}:0.4" ${REMOTE_DISCOVERY_SERVER_ID} C-m

# Examine in Pane 1: run AP container
echo "Pane 1: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.1" '. ~/workspace/banpaku_scripts/proto2/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.1" ${USER_PASS} C-m
sleep 1
echo "Pane 1: Starting AP service..."
tmux send-keys -t "${SESSION}:0.1" '. ~/workspace/banpaku_scripts/proto2/docker_run_ap.sh' C-m

# Examine in Pane 3: tf2_echo
echo "Pane 3: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.3" '. ~/workspace/banpaku_scripts/proto2/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.3" ${USER_PASS} C-m
sleep 1
echo "Pane 3: setup SUPER_CLIENT"
tmux send-keys -t "${SESSION}:0.3" '. ~/workspace/banpaku_scripts/proto2/docker_setup_commander.sh' C-m
sleep 1
echo "Pane 3: Starting tf2_echo for world->base_link ..."
tmux send-keys -t "${SESSION}:0.3" "ros2 run tf2_ros tf2_echo world ${HOSTNAME}/base_link" C-m

# Examine in Pane 5: commander
echo "Pane 5: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.5" '. ~/workspace/banpaku_scripts/proto2/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.5" ${USER_PASS} C-m
sleep 1
echo "Pane 5: setup SUPER_CLIENT"
tmux send-keys -t "${SESSION}:0.5" '. ~/workspace/banpaku_scripts/proto2/docker_setup_commander.sh' C-m
tmux send-keys -t "${SESSION}:0.5" 'ros2 node list' C-m

tmux attach-session -t "${SESSION}"

