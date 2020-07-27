#!/bin/bash
set -e

echo "On ${BLUEROV2_IP:=192.168.2.2}, pushing to bareclone..."
ssh ${JETSON_USER:=laodi}@$BLUEROV2_IP 'cd ~/Playground/ct_ws/src/ORCA_control; git push bareclone master'

echo "Pulling from the pi..."
git pull $JETSON_USER@$BLUEROV2_IP:orca-bluerov-bare master
