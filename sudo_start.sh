WIZZY_HOME=/home/wizzy-aux/wizzy/

# for usb relay
chmod +x /dev/Relay

# init LED controller
# python3 $WIZZY_HOME/raspberry_ws/src/wizzybug_hmi/src/wizzy_led_controller.py & 

# initialize sound server
python3 $WIZZY_HOME/raspberry_ws/src/wizzybug_hmi/src/sound_server.py 6666 &

# FLIC button 
FLIC_DIR=$WIZZY_HOME/raspberry_ws/src/wizzybug_control/src/flic_button

# start flic server
$FLIC_DIR/flic_server/bin/armv6l/flicd -f  $FLIC_DIR/flic_server/bin/armv6l/wizzyButton.sqlite  -h hci1 &


