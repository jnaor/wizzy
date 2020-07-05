import vlc
import time

mrl='ding_dong.mp3'
i = vlc.Instance('--verbose 9 --vout omxil_vout'.split())
p = vlc.MediaPlayer()
p.stop()
media = i.media_new(mrl)
p.set_media(media)

#em = p.event_manager()
#em.event_attach(vlc.EventType.MediaPlayerPositionChanged, poschanged)
#em.event_attach(vlc.EventType.MediaPlayerEndReached, end_callback)
p.play()
time.sleep (5)

exit(1)

vlc_Instance = vlc.Instance("--verbose 9")
player = vlc_Instance.media_player_new("ding_dong.mp3")

devices = []
mods = player.audio_output_device_enum()

if mods:
    mod = mods
    while mod:
        mod = mod.contents
        print("adding device {}".format(mod.device))	
        devices.append(mod.device)
        mod = mod.next
else:
    print("null blat")

exit(1)

dingdong_player = vlc.MediaPlayer("B5.wav", "--verbose 9") # ding_dong.mp3")
dingdong_player.audio_set_volume(100)
print('initialized player')

for ind in range(5):
    print('playing')
    dingdong_player.play()
    time.sleep(3)


