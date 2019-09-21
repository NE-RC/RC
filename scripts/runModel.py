#!/usr/bin/env python
import os, cv2, rospy
import tensorflow as tf
import tensorflow.keras as keras
import numpy as np

from std_msgs.msg import Float32

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
session = tf.Session(config = config)

def main():
	model_path = '/home/nvidia/catkin_ws/src/RC/scripts/model_cl_081919.h5'
	#model_path = '/home/nvidia/catkin_ws/src/RC/scripts/model_cl_092119.h5'
	new_size = (180, 135)

	model = keras.models.load_model(model_path)

	rospy.init_node('model_out', anonymous=True)
	pub = rospy.Publisher('model_out', Float32, queue_size=10)

	cap = cv2.VideoCapture(1)

	while not rospy.is_shutdown():
		ret, frame = cap.read()
		#cv2.imshow('frame', frame)
		resized = cv2.resize(frame, new_size)

		steer = model.predict(np.array([resized]) / 255.0)
		pub.publish(steer)

		print(steer)
		cv2.waitKey(50)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
