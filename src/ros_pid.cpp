#include "ros_pid.h"

ROSPIDController::ROSPIDController(ros::NodeHandle & nh){
  read_ros_params(nh);

  sensor_sub = nh.subscribe(sensor_topic, 100, &ROSPIDController::sensor_callback, this);
  output_pub = nh.advertise<std_msgs::Float64>(output_topic, 100);
  timer = nh.createTimer(ros::Duration(period), &ROSPIDController::timer_callback, this);

}

//Read params
void ROSPIDController::read_ros_params(ros::NodeHandle & nh){
  if (!ros::param::get("~kP", kP)){
    kP = 0.0;
  }
  if (!ros::param::get("~kI", kI)){
    kI = 0.0;
  }
  if (!ros::param::get("~kD", kD)){
    kD = 0.0;
  }
  if (!ros::param::get("~kF", kF)){
    kF = 0.0;
  }
  if (!ros::param::get("~period", period)){
    period = 1.0/20.0;
  }
  if (!ros::param::get("~max_output", max_output)){
    max_output = 1.0;
  }
  if (!ros::param::get("~min_output", min_output)){
    min_output = -1.0;
  }
  if (!ros::param::get("~tolerance", tolerance)){
    tolerance = 0.1;
  }
  if (!ros::param::get("~sensor_topic", sensor_topic)){
    sensor_topic = "sensor_input";
  }
  if (!ros::param::get("~output_topic", output_topic)){
    output_topic = "output_topic";
  }
}

void ROSPIDController::sensor_callback(const std_msgs::Float64 & msg){
  //whenever we receive a sensor message, push it into the stack
  sensor_stack.push_back(msg.data);
}

void ROSPIDController::timer_callback(const ros::TimerEvent & event){
  //if we don't have a target, disable the output for safety
  if (!has_target){
    std_msgs::Float64 to_pub;
    to_pub.data = 0.0;
    output_pub.publish(to_pub);
  }

  if (sensor_stack.empty()){
    ROS_INFO("Sensor stack empty! Will not publish new PID output.");
    return;
  }

  //fetch most recent sensor measurement
  double current = sensor_stack[sensor_stack.size() - 1];
  double pid_output = Update(current);

  std_msgs::Float64 to_pub;
  to_pub.data = pid_output;
  output_pub.publish(to_pub);

  sensor_stack.clear();
}
