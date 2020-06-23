echo "Starting ROS core"
roscore &

JETSON_IP=10.0.0.20
WIZZY_HOME=/home/wizzy/Documents/repo/wizzy


echo "Starting perception remotely on jetson"
ssh wizzy@${JETSON_IP} "cd ${WIZZY_HOME} && source start-jetson.sh && roslaunch wizzybug_vision perception.launch"
echo "Remote command error code:"
echo $?
echo "Initialization Done"


