#!/usr/bin/env bash
set -euo pipefail

# ==============================
USER_PASS="sony1234"
# ==============================

# ------------------------------------------------------------------
# ANSI escape sequence definitions for spinner
ESC=$(printf '\033')
CSI="${ESC}["
RESET="${CSI}0m"
GREEN="${CSI}32m"
ERASE_LINE="${CSI}2K"
HIDE_CURSOR="${CSI}?25l"
SHOW_CURSOR="${CSI}?25h"

# ------------------------------------------------------------------
# Spinner: shows animated indicator
# Usage: spinner "Message"
function spinner() {
  local msg="$*"
  local spin='⠧⠏⠛⠹⠼⠶'
  local n=${#spin}
  local i=0
  trap 'printf "%s${SHOW_CURSOR}\n" ""' EXIT
  while true; do
    printf "%s" "${ERASE_LINE}"
    printf "%s %s" "${GREEN}${spin:i++%n:1}${RESET}" "${msg}"
    printf "\r${HIDE_CURSOR}"
    sleep 0.1
  done
}

# Stop spinner and restore cursor visibility
function stop_spinner() {
  local pid=$1
  kill "${pid}" >/dev/null 2>&1 || true
  wait "${pid}" 2>/dev/null || true
  printf "%s" "${SHOW_CURSOR}"
}

# ------------------------------------------------------------------
# Configuration
SESSION="sensor_session"

# ------------------------------------------------------------------
# Cache sudo credentials once
echo "Requesting sudo credentials..."
sudo -v
echo "Sudo credentials updated."

echo
# ------------------------------------------------------------------
# Prevent duplicate tmux session
if tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "Session '${SESSION}' already exists. Exiting."
  exit 1
fi

echo "Creating tmux session '${SESSION}' with 2 vertical panes..."
# Create session with a single pane
tmux new-session -d -s "${SESSION}"
# Split window vertically into two equal panes
tmux split-window -v -t "${SESSION}:0.0"

echo "Pane layout ready."

# ------------------------------------------------------------------
# Pane 0: Start sensors and wait with spinner
echo "[Pane 0] Starting sensors..."
tmux send-keys -t "${SESSION}:0.0" \
  ". ~/workspace/banpaku_scripts/jetson/start_sensors.sh" C-m

# If sudo required inside script, send password
sleep 1
echo "[Pane 0] Sending sudo password..."
tmux send-keys -t "${SESSION}:0.0" "${USER_PASS}" C-m

echo "[Pane 0] Waiting for sensor init (15s)..."
(
  spinner "Initializing sensors..."
)&
SPINNER_PID=$!
sleep 15
stop_spinner "${SPINNER_PID}"
echo "[Pane 0] Sensors initialized."

# ------------------------------------------------------------------
# Pane 1: Set exposure and remove container
echo "[Pane 1] Setting exposure and removing container..."
tmux send-keys -t "${SESSION}:0.1" \
  ". ~/workspace/banpaku_scripts/jetson/set_exposure_and_remove_docker_container.sh" C-m

# If exposure script requires sudo, send password
sleep 1

echo "[Pane 1] Operation dispatched. Check pane logs for status."

# ------------------------------------------------------------------
# Attach to session
echo "Attaching to tmux session '${SESSION}'..."
tmux attach-session -t "${SESSION}"

