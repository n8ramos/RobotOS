#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

//variables to store velocity and angular velocity
double v, w = 0.0;

void callbackcmd_vel(const geometry_msgs::Twist& cmd_vel);

int main (int argc, char **argv) {	
	//initialize ros node controller
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;
	//subscriber to command velocities
	ros::Subscriber cmd_subscriber = n.subscribe("cmd_vel", 1, callbackcmd_vel);
	//publisher for pose data
	ros::Publisher pose_publisher = n.advertise<geometry_msgs::Pose2D>("pose2D", 1);
	//10 hz
	ros::Rate loop_rate(10);

	//variable to store pose data
	geometry_msgs::Pose2D pose_msg;
	//variable to store time data
	ros::Time current_time, last_time;
	//initialize time
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	//variables to store positional data
	double x, delta_x, y, delta_y, th, delta_th, dt = 0.0;

	//initialize pose values
	pose_msg.x = 0.0;
	pose_msg.y = 0.0;
	pose_msg.theta = 0.0;
	//initialize pose topic
	pose_publisher.publish(pose_msg);

	while(ros::ok())
	{
		//update data from callback
		ros::spinOnce();
		//update time
		current_time = ros::Time::now();
		//calculate delta values
		dt = (current_time - last_time).toSec();
		delta_x = v * cos(th) * dt;
		delta_y = v * sin(th) * dt;
		delta_th = w * dt;

		//calculate new pos
		x += delta_x;
		y += delta_y;
		th += delta_th;

		//publish calculated values
		pose_msg.x = x;
		pose_msg.y = y;
		pose_msg.theta = th*(180.0/3.141592653589793238463);
		ROS_INFO_STREAM("Publishing");
		pose_publisher.publish(pose_msg);
		last_time = current_time;
		loop_rate.sleep();
	}
	ros::shutdown();
	return 0;
}

void callbackcmd_vel(const geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO("Received the following command velocity value: %f, %f", cmd_vel.linear.x, cmd_vel.angular.z);
	//store angular velocity
	w = cmd_vel.angular.z;
	//store linear velocity
	v = cmd_vel.linear.x;
}