#!/usr/bin/env python

# system libraries
import time
import sys

# My Modules
from OurModules import functions_personality as pf

def main(argv):
	pf.sound_init()
	time.sleep(1)
	pf.play_sound_group(pf.sound_group_hello,100)


if __name__ == '__main__': # main loop
	try: # if no problems
		main(sys.argv[1:])
        
 	except Exception,e: # if a problem
 		print ("Error!")
		print(e)
		pass