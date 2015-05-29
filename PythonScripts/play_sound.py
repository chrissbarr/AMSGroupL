#!/usr/bin/env python

# Plays a sound file
# Written by Russell Brinkworth 2013

import pygame
import time
from os.path import expanduser

Sound_folder = '{0}/Documents/Sounds/'.format(expanduser('~')) # folder that contains the sound file

def play_sound(sound_file): # subroutine to play the specified sound file
    global Sound_folder
    if pygame.mixer.music.get_busy():
        pygame.mixer.fadeout(50) # fade out the current sound if is was one
    pygame.mixer.music.load(Sound_folder + sound_file)
    pygame.mixer.music.play()

def main():
    pygame.mixer.pre_init(44100, -16, 2, 4096) # set sound buffer. Prevents popping when playing sound files
    pygame.init()
    pygame.mixer.init()
    #pygame.mixer.Sound.set_volume(0.5)
    play_sound('Imperial_march.ogg')
    time.sleep(20)
    play_sound('VERY_good_time.ogg')
    time.sleep(3.5)
    pygame.mixer.quit()
    

if __name__ == '__main__':
    main()
