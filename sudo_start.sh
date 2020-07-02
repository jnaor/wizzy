# for usb relay (? don't remember)
chmod +x /dev/ttyACM0

# init LED controller
python3 wizzy_led_controller.py

# FLIC button 
FLIC_DIR=/home/wizzy-aux/wizzy/raspberry_ws/src/wizzybug_control/src/flic_button

# start flic server
$FLIC_DIR/flic_server/bin/armv6l/flicd -f  $FLIC_DIR/flic_server/bin/armv6l/wizzyButton.sqlite  -h hci1 &


