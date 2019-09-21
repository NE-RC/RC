#include <ros/ros.h>
#include <iostream>

#include "ros_pid.h"

int main(int argc, char* argv[]){

  ros::init(argc, argv, "velocity_controller_node");
  ros::NodeHandle node;

  //the PID controller will start automatically
  ROSPIDController pid_controller(node);

  //set an arbitrary target (for now)
  //pid_controller.SetTarget(1.0);

  ros::spin();

  return 0;
}

