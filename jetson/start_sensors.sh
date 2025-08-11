#!/bin/bash

sudo systemctl start systemd-timesyncd
sleep 5
sudo systemctl stop systemd-timesyncd

cd ~/workspace/aii-proto/jetson/device/imx471
make insmod
