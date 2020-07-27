#!/bin/bash

# Set up the repository clones on a blank raspberry pi.
# Prepare the directories expected by push2pi and pullfrompi.
# This requires an internet connection to clone from Github.

ssh ${JETSON_USER:=laodi}@${BLUEROV2_IP:=192.168.2.2} <<'ENDSSH'
git clone git@github.com:tsaoyu/ORCA_advance_control.git ~/Playground/ct_ws/src/ORCA_control
git clone --bare ~/Playground/ct_ws/src/ORCA_control orca-bluerov-advance-bare
cd ~/Playground/ct_ws/src/ORCA_control
git remote add bareclone ~/orca-bluerov-advance-bare
ENDSSH