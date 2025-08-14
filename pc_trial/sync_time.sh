#!/bin/bash
# usage: ./sync_time.sh user@remote_host

REMOTE=$1

if [ -z "$REMOTE" ]; then
    echo "Usage: $0 user@remote_host"
    exit 1
fi

# JSTの時刻を"YYYY-MM-DD HH:MM:SS"形式で取得
TIME_JST=$(TZ=Asia/Tokyo date -d "1 second" +"%Y-%m-%d %H:%M:%S")

echo "Local JST time: $TIME_JST"
echo "Updating time on $REMOTE ..."

# SSH先で時刻を反映
ssh -t "$REMOTE" "sudo date -s '$TIME_JST'"

echo "Time updated on $REMOTE."

