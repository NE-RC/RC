#!/usr/bin/env python

import os, cv2, rospy
import tensorflow as tf
import tensorflow.keras as keras
import numpy as np

from RC.msg import pwmout

def main():
	model_path = '/home/nick/catkin_ws/src/RC/scripts/NickModel.h5'
	directory = '/home/nick/ProgProjects/RC-ML/newData/frames/'
	new_size = (180, 135)

	model = keras.models.load_model(model_path)

	rospy.init_node('auto_drive', anonymous=True)
	pub = rospy.Publisher('pwmout', pwmout, queue_size=10)
	msg = pwmout()
	msg.throttle = 0.23

	for i in range(5800):
		filename = directory + str(i) + '.png'
		if os.path.isfile(filename):

			image = cv2.imread(filename)
			cv2.imshow('image', image)
			resized = cv2.resize(image, new_size)

			msg.steer = model.predict(np.array([resized]) / 255.0)
			pub.publish(msg)

			print(msg.steer)
			cv2.waitKey(50)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
