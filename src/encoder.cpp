#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <chrono>
#include <ctime>

ros::Publisher encoder_pub;

double prev_val = -1.0;
std::chrono::time_point<std::chrono::system_clock> prev_time, curr_time;
double ticks_per_rotation = 4096.0 * 2.0 / 3.0;

void callback(const std_msgs::Float32 & msg) {
	if (prev_val == -1.0) {
		prev_val = msg.data;
		prev_time = std::chrono::system_clock::now();
        } else {
		curr_time = std::chrono::system_clock::now(); 
		std::chrono::duration<double> elapsed = curr_time - prev_time; 	

		double dVal = msg.data - prev_val;
		double dt = elapsed.count();

		prev_val = msg.data;
		prev_time = curr_time;
		
		std_msgs::Float64 enc_vel;
		enc_vel.data = dVal / ticks_per_rotation / dt;
		encoder_pub.publish(enc_vel);
	}
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "Encoder");
	ros::NodeHandle nh;

	encoder_pub = nh.advertise<std_msgs::Float64>("/jetson_encoder",100);
	ros::Subscriber encoder_sub = nh.subscribe("/encoder", 100, callback);

	ros::spin();

	return 0;
}


