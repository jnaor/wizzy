
import os
import sys
import socket

from pydub import AudioSegment
from pydub.playback import play

# standard loopback interface address (localhost)
HOST = '127.0.0.1'

# command-line arguments
if len(sys.argv) != 2:
    print('Usage: python3 sound_server <port>')
    exit(1)

# initalize sound player dict
sounds = dict()

# for each sound file
for sound_file in sys.argv[1:-1]:
   # get the filename only from the initial file path.
   filename = os.path.basename(sound_file)

   # get filename and extension separately.
   (sound_message, ext) = os.path.splitext(filename)

   # player for this sound. doesn't work
   sounds[sound_message] = AudioSegment.from_file(sound_file)

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
            sound_message = data.decode()
            print(f'received {sound_message}')
            
            if sound_message not in sounds.values():
                print(sounds.values())
                continue

            # play sounds from saved sounds dictionary
            play(sounds[sound_message])

