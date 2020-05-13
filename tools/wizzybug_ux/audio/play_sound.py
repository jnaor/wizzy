import os

def play_wav(wav):
	if wav.endswith('.wav'):
		os.system('aplay ' + wav)

def play_B5():
	play_wav(os.getcwd()+'/B5.wav')


def play_Alert():
        play_wav(os.getcwd()+'/Alert_C1_Trimmed.wav')


