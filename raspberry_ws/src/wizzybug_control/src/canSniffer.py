import can
import os
import struct
import logging

INTERFACE = 'can0'
BITRATE = 105263

# empirically measured linear velocity in m/s
MAX_VELOCITY = 0.5

# empirically measured angular velocity in degrees/s
MAX_ANGULAR_VELOCITY = 40.0

# normalizaton constant (max value of reading)
MAX_VALUE = 100.0

def sign(x):
  return 1-(x<=0)

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

            # normalize according to measured joystick characteristics
            x, y = min(abs(x), MAX_VALUE) * sign(x), min(abs(y), MAX_VALUE) * sign(y)

            # notice sign flip to make forward motion positive
            velocity = (y / MAX_VALUE) * MAX_VELOCITY

            # angular velocity
            angular_velocity = (x / MAX_VALUE) * MAX_ANGULAR_VELOCITY

            logging.debug('vel: {}, ang: {}'.format(velocity, angular_velocity))

def disable_iface(iface):
    os.system('sudo ip link set down {}'.format(iface))

def enable_iface(iface, bitrate):
    os.system('sudo /sbin/ip link set {} up type can bitrate {}'.format(iface, bitrate))

def find_bitrate(iface):
    disable_iface(iface)
    enable_iface(iface, BITRATE)
    can_recv(iface)

if __name__== "__main__":

    # set logging level
    logging.basicConfig(level=logging.DEBUG)

    # run
    find_bitrate(INTERFACE)
