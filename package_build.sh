#!/bin/bash
set -e
./prepare.sh&
# setup ros environment
. /app/devel/setup.bash
python3 -m flask run --host=0.0.0.0&
export ROS_MASTER_URI=$ROBOT_ROS_MASTER_URI
export ROS_IP=$PC_ROS_IP
rosrun TeleopNetApp main.py
# kill $(jobs -p)
# wait
