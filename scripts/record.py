#!/usr/bin/env python
import rospy
import cv2
import os
from RC.msg import pwmout
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge, CvBridgeError

storage_path, frame_dir, label_dir = '','',''
frame = None
steer_val, i = 0, 0

#update steering values when changed
def callback_pwm(data):
	global steer_val
	steer_val = data.steer

#record steering value and 
def callback_joy(data):
	global i
	if data.buttons[9]:
		img_path = os.path.join(frame_dir, str(i) + '.png')
		label_path = os.path.join(label_dir, str(i) + '.txt')
		f = open(label_path, 'w')

		cv2.imwrite(img_path, frame)
		f.write(str(steer_val))
		f.close()
		i+=1

def main():
	
	global storage_path, frame_dir, label_dir, frame
	storage_path = rospy.get_param('record_node/storage_path', '/media/nvidia/F867-A38E/')
	display_feed = rospy.get_param('record_node/display_feed', False)	
	
	rospy.init_node('record', anonymous=True)
	
	#check if storage_path is a valid director (used to check if SD card is plugged in)
	if not os.path.isdir(storage_path):
		rospy.logerr('%s is not a valid directory. Exitting...', storage_path)

	else:
		#make directories to store images and labels or clear folders if they exist
		frame_dir = os.path.join(storage_path, 'frames')
		label_dir = os.path.join(storage_path, 'labels')

		for directory in [frame_dir, label_dir]:
			if os.path.isdir(directory):
				for filename in os.listdir(directory):
					file_path = os.path.join(directory, filename)
					os.unlink(file_path)
			else:
				os.mkdir(directory)
		
		#iniialize video capture, subscribers and publishers
		cap = cv2.VideoCapture(1)
		bridge = CvBridge()

		rospy.Subscriber('joy', Joy, callback_joy)
		rospy.Subscriber('pwmout', pwmout, callback_pwm)
		image_pub = rospy.Publisher('C920_feed', Image, queue_size=10)

		while not rospy.is_shutdown():
			ret, frame = cap.read()
			#frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		
			try:
				image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
			except CvBridgeError as e:
				rospy.logerr(e)
	
			#display camera feed if parameter is set
			if display_feed:
				cv2.imshow('frame', frame)
			cv2.waitKey(1)
		
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
