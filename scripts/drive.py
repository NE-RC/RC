#!/usr/bin/env python
import rospy, cv2
from RC.msg import pwmout
from sensor_msgs.msg import Joy


def callback(data):
	msg = pwmout()
	msg.steer = data.axes[2] * -1
	msg.throttle = data.axes[1] * max_throttle
	if data.buttons[10]:
		msg.throttle = static_throttle
	if abs(msg.throttle) < min_throttle:
		msg.throttle = 0
	
	pub.publish(msg)
	
	print str(msg.steer) + " " + str(msg.throttle)

def main():
	global pub, static_throttle, max_throttle, min_throttle
	pub = rospy.Publisher('pwmout', pwmout, queue_size=10)

	static_throttle = rospy.get_param('teleop_drive/static_throttle', 0.12)
	max_throttle = rospy.get_param('teleop_drive/max_throttle', 0.2)
	min_throttle = rospy.get_param('teleop_drive/min_throttle', 0.1)

	rospy.init_node('drive', anonymous=True)

	rospy.Subscriber("joy", Joy, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
