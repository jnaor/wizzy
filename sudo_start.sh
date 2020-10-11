WIZZY_HOME=/home/wizzy-aux/wizzy/

# for usb relay
chmod +x /dev/Relay

# FLIC button 
FLIC_DIR=$WIZZY_HOME/raspberry_ws/src/wizzybug_control/src/flic_button

# start flic server
$FLIC_DIR/flic_server/bin/armv6l/flicd -f  $FLIC_DIR/flic_server/bin/armv6l/wizzyButton.sqlite  -h hci1 &


