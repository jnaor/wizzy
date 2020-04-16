import can
import os
import struct

INTERFACE = 'can0'
BITRATE = 105263

def can_recv(iface):
    bus = can.interface.Bus(bustype='socketcan', channel=iface)
    y=0
    x=0 
    for msg in bus:
        if len(msg.data) == 6:
            data_array = struct.unpack('>6b', bytes(msg.data))
            y = data_array[4]
        elif len(msg.data) == 4:
            data_array = struct.unpack('>4b', bytes(msg.data))
            x = data_array[2]
            print('{},{}'.format(x,y))


def disable_iface(iface):
    os.system('sudo ip link set down {}'.format(iface))

def enable_iface(iface, bitrate):
    os.system('sudo /sbin/ip link set {} up type can bitrate {}'.format(iface, bitrate))

def find_bitrate(iface):
    disable_iface(iface)
    enable_iface(iface, BITRATE)
    can_recv(iface)

if __name__== "__main__":
    find_bitrate(INTERFACE)
