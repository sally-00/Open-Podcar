#!/usr/bin/env python3
import pygame, rospy
from std_msgs.msg import Float32

if __name__=='__main__':

	rospy.init_node('joystick', anonymous=True)
	pygame.init()
	pygame.joystick.init()
	steerpub = rospy.Publisher('/steering_cmd', Float32, queue_size=1)
	speedpub = rospy.Publisher('/speed_cmd', Float32, queue_size=1)

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
			steerpub.publish(axis0)
			speedpub.publish(axis1)
			
			print("Axis {} value: {:>6.3f}".format(0,axis0))
			print("Axis {} value: {:>6.3f}".format(1,axis1))
			
	pygame.quit()
