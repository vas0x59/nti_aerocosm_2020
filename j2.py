import pygame
import time 
pygame.joystick.init()

# while True:
#     g_keys = pygame.event.get()
#     print(g_keys)
joystick_count = pygame.joystick.get_count()
print(joystick_count)
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
print(joysticks)
joystick = joysticks[0]
joystick.init()
print(joystick.get_name())
print(joystick.get_numhats())
# pygame.joystick.Jo1
print(joystick.get_numaxes())
print(joystick.get_numbuttons())
while True:
    print([joystick.get_axis(i) for i in  range(6) ])
    time.sleep(0.01)