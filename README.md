NetApp to guarantee the QoS to a user over the 5G core using ROS

```
pip3 install uvloop httptools uvicorn fastapi fastapi_utils evolved5g
```
install evolved-API:

```
cd src/evolvedApi/
sudo python3 setup.py install
```

To install the package on ROS on the Teleop repository:

```
source /opt/ros/<ros1_distro>/setup.bash
catkin build
```

Run flask :
```
export FLASK_APP=<path_to_net_app>/TeleopNetApp/src/evolvedApi/evolvedApi/endpoint.py
export FLASK_ENV=development
python3 -m flask run --host=0.0.0.0
```

Run the NetApp :
```
rosrun TeleopNetApp main.py
```