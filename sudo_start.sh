# for usb relay (? don't remember)
chmod +x /dev/ttyACM0

# FLIC button 
FLIC_DIR=/home/wizzy-aux/wizzy/raspberry_ws/src/wizzybug_control/src/flic_button

# start server
$FLIC_DIR/flic_server/bin/armv6l/flicd -f  $FLIC_DIR/flic_server/bin/armv6l/wizzyButton.sqlite  -h hci1 &

# start client
python3 $FLIC_DIR/flic_client/wizzy_btn_client.py


