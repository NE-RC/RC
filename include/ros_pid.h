#include "pid.h"

#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

/**
 * ROS Wrapper around PIDController
**/
class ROSPIDController : public PIDController {

  private:
    ros::Subscriber sensor_sub;
    ros::Publisher output_pub;
    //Use a timer to make the publisher publish at a fixed rate.
    ros::Timer timer;

    std::string sensor_topic, output_topic;

    //store all the sensor from the subscriber on a stack,
    //since we want the most recent sensor value when we update the PID Controller.
    std::vector<double> sensor_stack;

    double period;

    void read_ros_params(ros::NodeHandle & nh);
    void sensor_callback(const std_msgs::Float64 & msg);
    void timer_callback(const ros::TimerEvent & event);

  public:
    ROSPIDController(ros::NodeHandle & nh);

};
