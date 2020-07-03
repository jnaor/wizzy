import vlc
import time

dingdong_player = vlc.MediaPlayer("ding_dong.mp3")

for ind in range(5):
    dingdong_player.play()
    time.sleep(3)


