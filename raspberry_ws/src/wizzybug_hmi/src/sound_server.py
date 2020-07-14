
import os
import sys
import socket

# does not work for some reason
# import vlc

# TODO: pre-load file
from playsound import playsound

# standard loopback interface address (localhost)
HOST = '127.0.0.1'

# command-line arguments
if len(sys.argv) != 2:
    print('Usage: python3 sound_server <port>')
    exit(1)

# initalize sound player dict
#sound_player = dict()

# for each sound file
#for sound_file in sys.argv[1:-1]:
#    # get the filename only from the initial file path.
#    filename = os.path.basename(sound_file)

#    # get filename and extension separately.
#    (sound, ext) = os.path.splitext(filename)

#    # player for this sound. doesn't work 
#    # sound_player[sound] = vlc.MediaPlayer(sound_file)
#    sound_player[sound] = sound_file

## start listening
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.bind((HOST, int(sys.argv[-1])))
    print(f'listening for UDP messages on {int(sys.argv[-1])}')
    sock.listen()
    conn, addr = sock.accept()
    with conn:
        print('connected by', addr)
        while True:
            data = conn.recv(1024)
            if not data:
                break

            # play received sound
            sound = data.decode()
            print(f'received {sound}')
            
#            if sound not in sound_player.values():
#                print(sound_player.values())
#                continue

            # sound_player[sound].play()
            if not os.path.exists(sound):
                print(f'no such file {sound}')
                continue

            playsound(sound)
