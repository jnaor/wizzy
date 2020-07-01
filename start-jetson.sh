source /opt/ros/melodic/setup.bash
source ~/realsense_ws/devel/setup.bash
source ~/Documents/repo/wizzy/jetson_ws/devel/setup.bash

export ROS_MASTER_URI=http://10.0.0.10:11311
export ROS_IP=10.0.0.20
roslaunch wizzybug_vision perception.launch
