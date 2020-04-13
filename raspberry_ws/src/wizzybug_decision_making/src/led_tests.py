#!/usr/bin/env python

import numpy as np
import time
import serial
import struct


def pack_led_ring_message(sections=[0, 0, 0, 0, 0, 0], red=0, green=0, blue=0, A=0, D=0, S=0, R=0,
                          attack_level=0, sustain_level=0, sequence_delay=0, flags=0):
    sections_string = ''
    for section in sections:
        sections_string = str(section) + sections_string
    sections_int = int(sections_string, 2)
    # Python2:
    # byte_sequence =  struct.pack('13c',chr(65),chr(65),chr(sections_int),chr(red),chr(green),chr(blue),chr(A),chr(D),chr(S),chr(R),chr(attack_level),chr(sustain_level),chr(flags))
    # Python3:
    byte_sequence = struct.pack('14c', bytes([65]), bytes([65]), bytes([sections_int]), bytes([red]), bytes([green]),
                                bytes([blue]), bytes([A]), bytes([D]), bytes([S]), bytes([R]), bytes([attack_level]),
                                bytes([sustain_level]), bytes([sequence_delay]), bytes([flags]))

    return byte_sequence


def pack_haptic_motors_message(motors=0, power=0, duration=0, repetiton=0, pulse_delay=0, sequence_delay=0, flags=0):
    motors_string = ''
    for motor in motors:
        motors_string = str(motor) + motors_string
    motors_int = int(motors_string, 2)

    # Python3:
    byte_sequence = struct.pack('9c', bytes([66]), bytes([66]), bytes([motors_int]), bytes([power]), bytes([duration]),
                                bytes([repetiton]), bytes([pulse_delay]), bytes([sequence_delay]), bytes([flags]))

    return byte_sequence

    # 1,[1,0,0,0,0,0],100,10,50,200,100,200,100,250,150,200,0
    # 2,[1,0,0,0,0,0],150,100,4,200,200,0


if __name__ == '__main__':
    try:
        serial_channel = serial.Serial('COM6', 9600, timeout=1)  # Automatically opens
        time.sleep(2)  # Must wait for arduino bootloader to load!

        test_type = 1

        while True:
            print('Please enter desired LED pattern in the following formats:')
            print('For LEDs, use:')
            print('1, sections[6], red, green, blue, A, D, S, R, attack_level, sustain_level, sequence_delay, flags')
            print('Example: 1,[1,0,0,0,0,0],100,10,50,200,100,200,100,250,150,200,0')
            print('For Motors, use:')
            print('2, motors[6], power, duration, repetition, pulse_delay, sequence_delay, flags')
            print('Example: 2,[1,0,0,0,0,0],150,100,4,200,200,0')

            # cmd = raw_input('Your pattern is: ')  # python2
            cmd = input('Your pattern is: ')  # python3
            test_type = int(cmd[0]);
            cmd = cmd[2:]
            unpacked = 'First number incorrect, please use 1 or 2'

            if test_type == 1:
                binary_output = eval('pack_led_ring_message(' + cmd + ')')
                unpacked = struct.unpack('14B', binary_output)

            if test_type == 2:
                binary_output = eval('pack_haptic_motors_message(' + cmd + ')')
                unpacked = struct.unpack('9B', binary_output)

            print(unpacked)
            serial_channel.write(binary_output)
            # print('Arduino Response: '+serial_channel.read(100))  # python2
            print('Arduino Response: ' + str(serial_channel.read(1000)))

    finally:
        serial_channel.close()
