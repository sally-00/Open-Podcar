#!/usr/bin/env python3
import pygame, rospy

if __name__=='__main__':

	pygame.init()
	pygame.joystick.init()

	while not rospy.is_shutdown():
		for event in pygame.event.get():
			if event.type == pygame.JOYBUTTONDOWN:
				print("Joystick button pressed.")
			elif event.type == pygame.JOYBUTTONUP:
				print("Joystick button released.")
		joystick_count = pygame.joystick.get_count()
		
		for i in range(joystick_count):
			joystick = pygame.joystick.Joystick(i)
			joystick.init()
			
			axis0 = joystick.get_axis(0) # x axis, -1(left) to 1(right)
			axis1 = joystick.get_axis(1) # y axis, -1(up) to 1(down)
			axis2 = joystick.get_axis(2)
			axis3 = joystick.get_axis(3)
			
			print("Axis {} value: {:>6.3f}".format(0,axis0))
			print("Axis {} value: {:>6.3f}".format(1,axis1))
			print("Axis {} value: {:>6.3f}".format(2,axis2))
			print("Axis {} value: {:>6.3f}".format(3,axis3))
			
	pygame.quit()
