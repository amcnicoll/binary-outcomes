import keyboard
import os

from mido import Message, MidiFile, MidiTrack

NOTE = 50

EVENT_LOOKUP = {'down'  :   'note_on',
                'up'    :   'note_off'}

print('Begin tapping, hit ESC to finish...')
record = keyboard.record(until='esc')

midi    = MidiFile(type=0, ticks_per_beat=500)  # 120 BPM default
track   = MidiTrack()
midi.tracks.append(track)

track.append(Message('program_change', program=23, time=0))

# Trim repeated events and final ESC
events  = []
for event in record[:-1]:
    if len(events) > 0:
        if events[-1].event_type == event.event_type:
            continue
    events.append(event)

print('Counted %d notes' % (len(events)/2))

# Start of first note
msg = Message('note_on', note=NOTE, velocity=127, time=0)

# Convert to MIDI
for (last, now) in zip(events[0:-1], events[1:]):

    # Keyboard event types are in seconds
    # MIDI time (ticks) is in milliseconds
    msg = Message(  EVENT_LOOKUP[now.event_type],
                    note=NOTE, velocity=127,
                    time=int((now.time - last.time) * 1000))

    track.append(msg)

try:
    os.remove('tap_track.mid')
except WindowsError:
    pass
midi.save('tap_track.mid')
