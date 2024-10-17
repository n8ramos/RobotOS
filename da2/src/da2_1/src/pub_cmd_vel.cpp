#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main (int argc, char **argv) 
{
	//initialize the publisher for command velocity
	ros::init(argc, argv, "pub_cmd_vel");
	ros::NodeHandle n;
	//topic name: cmd_vel
	ros::Publisher cmd_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	//2 hertz
	ros::Rate loop_rate(2);

	//seed our random function
	srand(time(0));

	//message vel_msg is of type Twist
	geometry_msgs::Twist vel_msg;

	while(ros::ok())
	{
		//linear velocity
		vel_msg.linear.x = 0.5 * double(rand())/double(RAND_MAX); 
		//angular velocity
		vel_msg.angular.z = 2*double(rand())/double(RAND_MAX)-1;
		//publish findings
		cmd_publisher.publish(vel_msg);

		//allow callbacks
		ros::spinOnce();
		//wait for next loop
		loop_rate.sleep();
	}
	//should never make it here, but just in case
	ros::shutdown();
	return 0;
}
