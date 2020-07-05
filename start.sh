echo "start.sh running" > log.txt

echo "wizzy bringup"
roslaunch wizzybug_decision_making wizzybug_online.launch &

JETSON_IP=10.0.0.20
WIZZY_HOME=/home/wizzy/Documents/repo/wizzy


#echo "Starting perception remotely on jetson"
#ssh wizzy@${JETSON_IP} "cd ${WIZZY_HOME} && source start-jetson.sh && roslaunch wizzybug_vision perception.launch"
#echo "Remote command error code:"
#echo $?
#echo "Initialization Done"

# start flic client
echo "Starting FLIC button client"
FLIC_DIR=/home/wizzy-aux/wizzy/raspberry_ws/src/wizzybug_control/src/flic_button
python3 $FLIC_DIR/flic_client/wizzy_btn_client.py &

