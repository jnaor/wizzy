cd /home/user/wizzy/raspberry_ws
catkin config --init --extend /opt/ros/melodic
catkin build
source devel/setup.bash

cd ../jetson_ws
catkin config --init --extend /home/user/wizzy/raspberry_ws
catkin build
source devel/setup.bash

cd ../simulation_ws
catkin config --init --extend /home/user/wizzy/jetson_ws
catkin build
source devel/setup.bash
cd ..

source simulation_ws/devel/setup.bash

rosdep update

#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/user/wizzy/raspberry_ws/src:/home/user/wizzy/simulation_ws/src
#echo "added to ros package path"