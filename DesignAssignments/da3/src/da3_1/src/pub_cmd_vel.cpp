#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main (int argc, char **argv) 
{
	//initialize the publisher for command velocity
	ros::init(argc, argv, "command_pub");
	ros::NodeHandle n;
	//topic name: cmd_vel
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	//2 hertz
	ros::Rate loop_rate(2);

	//seed our random function
	srand(time(0));

	//message vel_msg is of type Twist
	geometry_msgs::Twist vel_msg;
	//init vel_msg
	vel_msg.linear.x = 2 * double(rand())/double(RAND_MAX);
	
	//make sure our speed is greater than 0.5
	if (vel_msg.linear.x < 0.5) {
		vel_msg.linear.x = vel_msg.linear.x + 0.5;
	}
	//use y for our goal distance
	vel_msg.linear.y = 5 * double(rand())/double(RAND_MAX);
	//make sure our distance is greater than 1
	if (vel_msg.linear.y < 1) {
		vel_msg.linear.y = vel_msg.linear.y + 1;
	}
	vel_msg.linear.z = 0.0;
	vel_msg.angular.x = 0.0;
	vel_msg.angular.y = 0.0;
	vel_msg.angular.z = 0.0;

	//variable to initiate a delay of 5 seconds
	int count = 0;

	while(ros::ok())
	{
		if (count >= 10) {
			//publish data
			ROS_INFO("\n\n\n******START PUBLISHING****************************\n");
			ROS_INFO_STREAM("Linear Velocity: " << vel_msg.linear.x);
			ROS_INFO_STREAM("Desired Distance: " << vel_msg.linear.y);
			ROS_INFO("To publish new data, hit control c and re-run both launch files.\n");
			ROS_INFO("\n\n\n**************************************************\n");
			//publish goal pose
			cmd_pub.publish(vel_msg);
		}
		else {
			count++;
		}
		//wait for next loop
		loop_rate.sleep();
	}
	//should never make it here, but just in case
	ros::shutdown();
	return 0;
}
