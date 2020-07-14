
import os
import sys
import socket

import vlc

# standard loopback interface address (localhost)
HOST = '127.0.0.1'

# command-line arguments
if len(sys.argv) < 3:
    print('Usage: python3 sound_server <sound_file_1> ... <sound_file_n> <port>')
    exit(1)

# initalize sound player dict
sound_player = dict()

# for each sound file
for sound_file in sys.argv[1:-1]:
    # get the filename only from the initial file path.
    filename = os.path.basename(sound_file)

    # get filename and extension separately.
    (sound, ext) = os.path.splitext(filename)

    # player for this sound
    sound_player[sound] = vlc.MediaPlayer(sound_file)

# start listening
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
            sound_player[sound].play()
