#!/usr/bin/env bash
set -euo pipefail

# ==============================
USER_PASS="sony1234"
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

# Split window into 3 horizontal sections: top 33%, middle 33%, bottom 34%
tmux split-window -v -t "${SESSION}" -p 66
tmux split-window -v -t "${SESSION}" -p 50

echo "Created TMUX session '${SESSION}' with 3 equal panes."

echo

# Examine in Pane 0: run AP container
echo "Pane 0: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.0" '. ~/workspace/banpaku_scripts/proto2/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.0" ${USER_PASS} C-m
sleep 1
echo "Pane 0: Starting AP service..."
tmux send-keys -t "${SESSION}:0.0" '. ~/workspace/banpaku_scripts/proto2/docker_run_ap.sh' C-m

# Examine in Pane 1: tf2_echo
echo "Pane 1: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.1" '. ~/workspace/banpaku_scripts/proto2/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.1" ${USER_PASS} C-m
sleep 1
echo "Pane 1: setup SUPER_CLIENT"
tmux send-keys -t "${SESSION}:0.1" '. ~/workspace/banpaku_scripts/proto2/docker_setup_commander.sh' C-m
sleep 1
echo "Pane 1: Starting tf2_echo for world->base_link ..."
tmux send-keys -t "${SESSION}:0.1" "ros2 run tf2_ros tf2_echo world ${HOSTNAME}/base_link" C-m

# Examine in Pane 2: commander
echo "Pane 2: Launching Docker environment..."
tmux send-keys -t "${SESSION}:0.2" '. ~/workspace/banpaku_scripts/proto2/run_docker.sh' C-m
sleep 1
tmux send-keys -t "${SESSION}:0.2" ${USER_PASS} C-m
sleep 1
echo "Pane 2: setup SUPER_CLIENT"
tmux send-keys -t "${SESSION}:0.2" '. ~/workspace/banpaku_scripts/proto2/docker_setup_commander.sh' C-m
tmux send-keys -t "${SESSION}:0.2" 'ros2 node list' C-m

tmux attach-session -t "${SESSION}"

