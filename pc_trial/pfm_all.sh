#!/bin/bash
set -euo pipefail

# ==============================
USER_PASS="sony1234"
REMOTE_DISCOVERY_SERVER_ID="2"
PROTO1X_DISCOVERY_GUID="44.53.01.5f.45.50.52.4f.53.49.4d.41"
#PROTO1X_IP_ADDRESS="43.21.213.16"
PROTO1X_IP_ADDRESS="192.168.10.105"
PROTO2_DISCOVERY_GUID="44.53.03.5f.45.50.52.4f.53.49.4d.41"
#PROTO2_IP_ADDRESS="43.21.214.161"
PROTO2_IP_ADDRESS="192.168.10.101"
# ==============================

# ANSI escape sequence definitions
ESC=$(printf '\033')
CSI="${ESC}["
RESET="${CSI}0m"
GREEN="${CSI}32m"
ERASE_LINE="${CSI}2K"
HIDE_CURSOR="${CSI}?25l"
SHOW_CURSOR="${CSI}?25h"

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

#!/bin/bash
# 新規セッション作成
tmux new-session -d -s "$SESSION" -n main

# 縦に3分割（even-verticalで均等化）
tmux split-window -v -t "$SESSION:0"
tmux split-window -v -t "$SESSION:0"
tmux select-layout -t "$SESSION:0" even-vertical

# 各行を横に分割
tmux split-window -h -t "$SESSION:0.0"
tmux split-window -h -t "$SESSION:0.1"
tmux split-window -h -t "$SESSION:0.2"

# 均等配置（縦横とも）
tmux select-layout -t "$SESSION:0" tiled

# ペイン名
tmux select-pane -t "$SESSION:0.0" -T "Pane 1"
tmux select-pane -t "$SESSION:0.1" -T "Pane 2"
tmux select-pane -t "$SESSION:0.2" -T "Pane 3"
tmux select-pane -t "$SESSION:0.3" -T "Pane 4"
tmux select-pane -t "$SESSION:0.4" -T "Pane 5"
tmux select-pane -t "$SESSION:0.5" -T "Pane 6"

echo "Created TMUX session '${SESSION}' with 6 equal panes."

## Pane 0: start discovery server local with spinner
echo "Pane 0: starting discovery server..."
tmux send-keys -t "${SESSION}:0.0" '. ~/workspace/banpaku_scripts/pc/run_pfm_docker.sh' C-m

echo -n "Waiting for docker (7s)... "
(
  spinner "Initializing docker..."
)&
SPINNER_PID=$!
sleep 8
stop_spinner "${SPINNER_PID}"
echo -e "${GREEN}Pane 0: Docker initialized.${RESET}"
tmux send-keys -t "${SESSION}:0.0" 'cd ros2_ws/src/launch_ros/; . ./fastrtps_config/run_discovery_server_local.sh' C-m


## Pane 1: start discovery server local with spinner
echo "Pane 1: starting discovery server..."
tmux send-keys -t "${SESSION}:0.1" '. ~/workspace/banpaku_scripts/pc/run_pfm_docker.sh' C-m

echo -n "Waiting for docker (7s)... "
(
  spinner "Initializing docker..."
)&
SPINNER_PID=$!
sleep 8
stop_spinner "${SPINNER_PID}"
echo -e "${GREEN}Pane 1: Docker initialized.${RESET}"
tmux send-keys -t "${SESSION}:0.1" 'cd ros2_ws/src/launch_ros/; . ./fastrtps_config/run_discovery_server_remote.sh' C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.1" '2' C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.1" ${PROTO1X_DISCOVERY_GUID} C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.1" ${PROTO1X_IP_ADDRESS} C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.1" '11812' C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.1" ${PROTO2_DISCOVERY_GUID} C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.1" ${PROTO2_IP_ADDRESS} C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.1" '11812' C-m
sleep 0.5

tmux send-keys -t "${SESSION}:0.1" ${REMOTE_DISCOVERY_SERVER_ID} C-m


## Pane 2: start PFM
echo "Pane 2: Starting PFM..."
tmux send-keys -t "${SESSION}:0.2" '. ~/workspace/banpaku_scripts/pc/run_pfm_docker.sh' C-m

echo -n "Waiting for docker (7s)... "
(
  spinner "Initializing docker..."
)&
SPINNER_PID=$!
sleep 8
stop_spinner "${SPINNER_PID}"
echo -e "${GREEN}Pane 2: Docker initialized.${RESET}"

echo "Pane 2: starting discovery client.."
tmux send-keys -t "${SESSION}:0.2" 'cd ros2_ws/src/launch_ros/; . ./fastrtps_config/set_discovery_client.sh' C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.2" 'source ../../install/setup.bash' C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.2" 'cd launch' C-m
#sleep 0.5
#tmux send-keys -t "${SESSION}:0.2" 'ros2 launch aibo_map_vdetr.launch.py' C-m


## Pane 3: start rviz
echo "Pane 3: "
tmux send-keys -t "${SESSION}:0.3" '. ~/workspace/banpaku_scripts/pc/run_pfm_docker.sh' C-m

echo -n "Waiting for docker (7s)... "
(
  spinner "Initializing docker..."
)&
SPINNER_PID=$!
sleep 8
stop_spinner "${SPINNER_PID}"
echo -e "${GREEN}Pane 3: Docker initialized.${RESET}"

echo "Pane 3: rviz"
tmux send-keys -t "${SESSION}:0.3" 'cd ros2_ws/src/launch_ros/; . ./fastrtps_config/set_discovery_super_client.sh' C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.3" 'source ../../install/setup.bash' C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.3" 'cd /home/ros-humble/ros2_ws/src/launch_ros/rviz' C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.3" 'ls' C-m


## Pane 4: convienience docker
echo "Pane 4: convinience pane"
tmux send-keys -t "${SESSION}:0.4" '. ~/workspace/banpaku_scripts/pc/run_pfm_docker.sh' C-m

echo -n "Waiting for docker (7s)... "
(
  spinner "Initializing docker..."
)&
SPINNER_PID=$!
sleep 8
stop_spinner "${SPINNER_PID}"
echo -e "${GREEN}Pane 4: Docker initialized.${RESET}"

echo "Pane 3: "
tmux send-keys -t "${SESSION}:0.4" 'cd ros2_ws/src/launch_ros/; . ./fastrtps_config/set_discovery_super_client.sh' C-m
sleep 0.5
tmux send-keys -t "${SESSION}:0.4" 'source ../../install/setup.bash' C-m
tmux send-keys -t "${SESSION}:0.4" 'ls' C-m


tmux attach -t "$SESSION"


