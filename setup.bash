cd raspberry_ws
catkin config --init --extend /opt/ros/melodic
catkin build
source devel/setup.bash

cd ../jetson_ws
catkin config --init --extend ../raspberry_ws
catkin build
source devel/setup.bash

cd ../simulation_ws
catkin config --init --extend ../jetson_ws
catkin build
source devel/setup.bash
cd ..

source simulation_ws/devel/setup.bash

# haven't figured out when this is actually needed. uncomment if you encounter trouble
# rosdep update
