#!/usr/bin/env python
import rospy
from RC.msg import pwmout

def talker():
	pub = rospy.Publisher('pwmout', pwmout)
	rospy.init_node('drive', anonymous=True)
	r = rospy.Rate(10)
	msg = pwmout()
	msg.steer = -1
	msg.throttle = -0.5

	while not rospy.is_shutdown():
		pub.publish(msg)
		r.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
