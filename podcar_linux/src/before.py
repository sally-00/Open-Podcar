#!/usr/bin/env python3
import pygame, rospy
from std_msgs.msg import Int16

class control_with_joystick:

	def __init__(self):

		self.speed_pub = rospy.Publisher('/speed_cmd', Int16, queue_size=10)
		self.steer_pub = rospy.Publisher('/steering_cmd', Int16, queue_size=10)

		pygame.init()
		pygame.joystick.init()

	def start_control(self):
		
		while not rospy.is_shutdown():
			joystick_count = pygame.joystick.get_count()
			for i in range(joystick_count):
				joystick = pygame.joystick.Joystick(i)
				joystick.init()
				#for event in pygame.event.get():
				#	if event.type == pygame.JOYBUTTONDOWN:
				#		print("Joystick button pressed.")
				#	elif event.type == pygame.JOYBUTTONUP:
				#		print("Joystick button released.")
				'''
				iter_num = 10
				axis0_sum = 0
				axis1_sum = 0
				for i in range(iter_num):
					axis0_sum = axis0_sum + joystick.get_axis(0) # x axis, -1(left) to 1(right)
					axis1_sum = axis1_sum + joystick.get_axis(1) # y axis, -1(up) to 1(down)
				axis0 = axis0_sum/iter_num
				axis1 = axis1_sum/iter_num
				'''
				axis0 = joystick.get_axis(0) # x axis, -1(left) to 1(right)
				axis1 = joystick.get_axis(1) # y axis, -1(up) to 1(down)

				print("Axis {} value: {:>6.3f}".format(0,axis0))
				print("Axis {} value: {:>6.3f}".format(1,axis1))

				#speed = self.map_value(axis0, -1, 1, -255, 255)
				#steer = self.map_value(axis1, -1, 1, -255, 255)
				#rospy.loginfo("Speed: %s, Steer: %s.", speed, steer)
				#self.speed_pub.publish(speed)
				#self.steer_pub.publish(steer)
				
		pygame.quit()

	def map_value(self, value, min1, max1, min2, max2):
		range1 = max1 - min1
		range2 = max2 - min2
		scaled01 = (value - min1)/range1
		result = min2 + scaled01*range2
		return result



if __name__ == '__main__':
	rospy.init_node('control_with_joystick', anonymous=True)
	try:
		cj = control_with_joystick()
		cj.start_control()
	except rospy.ROSInterruptException:
		pass
