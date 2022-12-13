#!/bin/bash
set -e
./prepare.sh&
# setup ros environment
. /app/devel/setup.bash
python3 -m flask run --host=0.0.0.0 -p 5003
rosrun TeleopNetApp main.py
kill $(jobs -p)
wait
