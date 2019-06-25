import glob
import time
import serial
from platform import uname
from pygame import mixer

IS_WINDOWS = uname()[0] == 'Windows'

if IS_WINDOWS:
    print('Running Windows; not parsing serial.')
else:
    print('Opening USB serial port...')
    try:
        port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=5)
    except:
        print('Could not open port... check USB?')
        exit(-1)

print('Locating tracks...')

tracks = glob.glob('tracks/*.wav')

for track in tracks:
    print('\t' + track)

print('Starting up pygame mixer...')

mixer.init(frequency=22050,
           size=-16,
           channels=2,
           buffer=2**13)

mixer.set_num_channels(len(tracks))
mixer.set_reserved(len(tracks))

sounds      = []
channels    = []

for channel_num, track in enumerate(tracks):
    sound   = mixer.Sound(track)
    channel = mixer.Channel(channel_num)

    sounds.append(sound)
    channels.append(channel)

track_duration = sound.get_length()
start = time.time() - track_duration

while True:
    if time.time() > (start + track_duration):
        print('(Re)starting tracks...')
        for channel in channels:
            channel.stop()
        for channel, sound in zip(channels, sounds):
            channel.play(sound)
        start = time.time()

    if IS_WINDOWS:
        continue

    # Quick and dirty - never use a newline (\n) on Arduino side
    # so that we can use it as our break character here
    try:
        bites = port.read_until('\n')
    except:
        print('No volume updates received')
        continue

    channel_num = bites[0]
    volume_byte = bites[1]
    if channel_num < len(channels):
        channels[channel_num].set_volume(volume_byte / 255.0)
    else:
        print('Invalid track byte received!')
