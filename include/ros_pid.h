#include "pid.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>

/**
 * ROS Wrapper around PIDController
**/
class ROSPIDController : public PIDController {

  private:
    ros::Subscriber sensor_sub;
    ros::Publisher output_pub;
    ros::Publisher setpoint_pub;
    //Use a timer to make the publisher publish at a fixed rate.
    ros::Timer timer;

    std::string sensor_topic, output_topic, setpoint_topic;

    double period;
    double last_sensor_reading;

    void read_ros_params(ros::NodeHandle & nh);
    void sensor_callback(const std_msgs::Float64 & msg);
    void timer_callback(const ros::TimerEvent & event);

  public:
    ROSPIDController(ros::NodeHandle & nh);

};
