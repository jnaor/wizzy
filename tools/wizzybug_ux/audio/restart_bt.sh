#!/bin/sh
sudo killall pulseaudio
sudo killall bluetoothd
sudo killall blueman-applet
killall obexd
sudo /usr/lib/bluetooth/bluetoothd -d &
pulseaudio --start
blueman-applet &
