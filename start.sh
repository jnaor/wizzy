
JETSON_IP=10.0.0.20
WIZZY_HOME=/home/wizzy/Documents/repo/wizzy

ssh wizzy@${JETSON_IP} "cd ${WIZZY_HOME} && source start-jetson.sh && roslaunch wizzybug_vision perception.launch"


