# start roscore on raspberry
roscore &

JETSON_IP=10.0.0.20
ssh wizzy@${JETSON_IP}
export ROS_MASTER_URI=http://10.0.0.10:11311
export ROS_IP=http://10.0.0.20
roslaunch wizzybug_vision perception.launch
