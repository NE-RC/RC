#!/usr/bin/env python
import rospy
from RC.msg import pwmout
from sensor_msgs.msg import Joy

def callback(data):
	msg = pwmout()
	msg.steer = data.axes[2] * -1
	msg.throttle = data.axes[1]
	print str(msg.steer) + " " + str(msg.throttle)
	pub.publish(msg)

def main():
	global pub
	pub = rospy.Publisher('pwmout', pwmout, queue_size=10)

	rospy.init_node('drive', anonymous=True)

	rospy.Subscriber("joy", Joy, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
