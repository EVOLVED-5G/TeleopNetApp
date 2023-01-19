#!/bin/bash
set -e
./prepare.sh&
# setup ros environment
. /app/devel/setup.bash
python3 -m flask run --host=0.0.0.0&
rosrun TeleopNetApp main.py
# kill $(jobs -p)
# wait
