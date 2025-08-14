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

SESSION="ap"
# Create session
tmux new-session -d -s "${SESSION}"
tmux split-window -v -t "${SESSION}"

echo "Created TMUX session '${SESSION}' with 2 equal panes."

echo

# Examine in Pane 0: launch container and run local discovery server
echo "Pane 0: Launching Docker..."
tmux send-keys -t "${SESSION}:0.0" '. ~/workspace/banpaku_scripts/proto2/run_docker.sh' C-m
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
tmux send-keys -t "${SESSION}:0.0" ". ~/workspace/banpaku_scripts/proto2/docker_run_local_discovery_server.sh" C-m

# Examine in Pane 1: run remote discovery server
echo "Pane 1: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.1" '. ~/workspace/banpaku_scripts/proto2/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.1" ${USER_PASS} C-m
sleep 1
echo "Pane 1: Starting remote discovery server..."
tmux send-keys -t "${SESSION}:0.1" '. ~/workspace/banpaku_scripts/proto2/docker_run_remote_discovery_server.sh' C-m
tmux send-keys -t "${SESSION}:0.1" '0' C-m
tmux send-keys -t "${SESSION}:0.1" ${REMOTE_DISCOVERY_SERVER_ID} C-m

tmux attach-session -t "${SESSION}"

