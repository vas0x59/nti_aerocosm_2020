#!/usr/bin/env python3
import inputs
print(inputs.devices)
# pads = inputs.devices.gamepads

# if len(pads) == 0:
    # raise Exception("Couldn't find any Gamepads!")

while True:
    events = inputs.get_gamepad()
    for event in events:
        print(event.code, event.state)
# left joy - ABS_X,ABS_Y