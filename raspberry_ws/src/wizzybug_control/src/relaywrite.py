"""
From :
https://github.com/numato/samplecode/blob/master/RelayAndGPIOModules/USBRelayAndGPIOModules/python/usbrelay1_2_4_8/relaywrite.py
"""


import sys
import serial

if (len(sys.argv) < 2):
	print "Usage: relaywrite.py <PORT> <RELAYNUM> <CMD>\nEg: relaywrite.py COM1 0 on"
	sys.exit(0)
else:
	portName = sys.argv[1];
	relayNum = sys.argv[2];
	relayCmd = sys.argv[3];

#Open port for communication
serPort = serial.Serial(portName, 19200, timeout=1)

#Send the command
serPort.write("relay "+ str(relayCmd) +" "+ str(relayNum) + "\n\r")

print "Command sent..."

#Close the port
serPort.close()

