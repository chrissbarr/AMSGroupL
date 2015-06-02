#!/usr/bin/env python

"""
This script is a compilation of functions. 

These functions were originally included in several of the other scripts, but it became easier to consolidate them all in one place to avoid code duplication.

Most of these functions relate to "personality" elements of the software - sound playback and such. 
Supporting functions related to or used with these functions are probably also here, because I'm lazy.

"""

# system libraries
import time
import select
import sys
import socket
import os
import math
import pygame
from os.path import expanduser

import random
from random import randint




# SOUNDS
sound_folder = '{0}/Documents/Sounds/PortalTurret/'.format(expanduser('~')) # folder that contains the sound file
sound_group_startup = ['turret_deploy_2.ogg','turret_deploy_4.ogg']
sound_group_shutdown = ['turret_disabled_4.ogg','turret_retire_1.ogg','turret_retire_2.ogg','turret_retire_4.ogg','turret_retire_5.ogg','turret_retire_6.ogg','turret_retire_7.ogg']
sound_group_error = ['turret_disabled_2.ogg','turret_tipped_2.ogg','turret_tipped_3.ogg','turret_tipped_4.ogg']
sound_group_search = ['turret_search_4.ogg','turret_autosearch_2.ogg','turret_autosearch_3.ogg']
sound_group_found = ['turret_active_6.ogg','turret_active_7.ogg','turret_active_8.ogg','sp_sabotage_factory_good_prerange01.ogg']
sound_group_pickup = ['turret_pickup_3.ogg','turret_pickup_8.ogg','turret_pickup_7.ogg','turret_pickup_10.ogg','turretlaunched03.ogg','turretlightbridgeblock03.ogg']
sound_group_push = ['turretsquashed04.ogg','turretsquashed06.ogg','turretshotbylaser07.ogg']
sound_group_ping = ['ping.ogg']
sound_group_lonely = ['turret_search_1.ogg','turret_autosearch_5.ogg']

last_played_sound = ['none','none','none']

def sound_init():
	pygame.mixer.pre_init(44100, -16, 2, 4096) # set sound buffer. Prevents popping when playing sound files
	pygame.init()
	pygame.mixer.init()
		
def play_sound(sound_file): # subroutine to play the specified sound file
	global sound_folder
	if pygame.mixer.music.get_busy():
		pygame.mixer.fadeout(50) # fade out the current sound if is was one
	pygame.mixer.music.load(sound_folder + sound_file)
	pygame.mixer.music.play()
    
def play_sound_group(sound_group, probability):
	global last_played_sound
	play_sound_probability = randint(0,100)
	if(play_sound_probability <= probability):
		selected_sound_index = randint(0,len(sound_group)-1)

		# if we have more than one sound option
		if(len(sound_group) > 1):
			# make sure we don't play the same clip twice in a row
			while(sound_group[selected_sound_index] == last_played_sound[0]):
				selected_sound_index = randint(0,len(sound_group)-1)

		new_sound = sound_group[selected_sound_index]

		last_played_sound[0], last_played_sound[1], last_played_sound[2] = new_sound, last_played_sound[0], last_played_sound[1] # primitive list of last 3 played sounds
		
		play_sound(new_sound)

def block_wait_sound_finish():
	while (pygame.mixer.music.get_busy()):	# wait for sound to stop playing
			time.sleep(.1)

def main(argv):
	sound_init()
	
	

if __name__ == '__main__': # main loop
	try: # if no problems
		main(sys.argv[1:])
        
 	except Exception,e: # if a problem
 		print ("Error!")
		print(e)
		play_sound_group(sound_group_error,100)
		block_wait_sound_finish()
		pass
 
