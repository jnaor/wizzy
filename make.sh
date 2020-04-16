cd jetson_ws
catkin_make
cd ../raspberry_ws
catkin_make
cd ../simulation_ws
catkin_make
cd ..

source jetson_ws/devel/setup.bash

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/user/wizzy/raspberry_ws/src:/home/user/wizzy/simulation_ws/src
echo "added to ros package path"