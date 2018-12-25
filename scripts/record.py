#!/usr/bin/env python
import rospy
import cv2
from RC.msg import pwmout
from sensor_msgs.msg import Joy

frame = None
steer_val = 0

def callback_pwm(data):
	global steer_val
	steer_val = data.steer

def callback_joy(data):
	if data.axes[4] < 0:
		cv2.imwrite('1.png', frame)
		#print steer_val

def main():
	global frame
	cap = cv2.VideoCapture(1)

	rospy.init_node('record', anonymous=True)
	rospy.Subscriber("joy", Joy, callback_joy)
	rospy.Subscriber('pwmout', pwmout, callback_pwm)
	while not rospy.is_shutdown():
		ret, frame = cap.read()
		#gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		cv2.imshow('frame', frame)
		cv2.waitKey(1)
	
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
