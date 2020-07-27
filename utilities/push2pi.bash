#!/bin/bash
set -e

echo "Pushing to the robot at ${BLUEROV2_IP:=192.168.2.2}..."
git push ${JETSON_USER:=laodi}@$BLUEROV2_IP:orca-bluerov-bare


echo "Pulling from bareclone on the robot..."
ssh $JETSON_USER@$BLUEROV2_IP 'cd ~/Playground/ct_ws/src/ORCA_control; git pull bareclone master'
