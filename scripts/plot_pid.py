#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt

from std_msgs.msg import Float64

setpoint_data = []
sensor_data = []

def main():
    rospy.init_node("pid_plotter", anonymous=True)

    setpoint_topic = rospy.get_param("~pid_setpoint_topic", "pid_setpoint_topic")
    sensor_topic = rospy.get_param("~pid_sensor_topic", "pid_sensor_topic")

    rospy.Subscriber(setpoint_topic, Float64, sensor_callback)
    rospy.Subscriber(sensor_topic, Float64, setpoint_callback)

    print("Waiting for {} topic...".format(setpoint_topic))
    rospy.wait_for_message(setpoint_topic, Float64)
    print("Waiting for {} topic...".format(sensor_topic))
    rospy.wait_for_message(sensor_topic, Float64)

    print("Got all topics. Spinning!")

    plt.ion()
    fig = plt.figure()

    while not rospy.is_shutdown():
        plt.plot(sensor_data, color="blue")
        plt.plot(setpoint_data, color="red")

        fig.canvas.draw()
        fig.canvas.flush_events()
        rospy.rostime.wallsleep(0.5)

def setpoint_callback(msg):
    setpoint_data.append(msg.data)

def sensor_callback(msg):
    sensor_data.append(msg.data)

if __name__ == '__main__':
    main()
