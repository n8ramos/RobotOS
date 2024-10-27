#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
const double pi = 3.141592653589793238463;
int main (int argc, char **argv)
{
	//initialize the publisher for pose
	ros::init(argc, argv, "command_pose");
	ros::NodeHandle n;
	//topic name: goto_pose
	ros::Publisher cmd_pose = n.advertise<geometry_msgs::Pose2D>("goto_pose", 1000);
	//2 hertz
	ros::Rate loop_rate(2);
	//message goto_pose is of type Pose2D
	geometry_msgs::Pose2D goto_pose;
	//seed our random function
	srand(time(0));
	//generate random pose
	goto_pose.x = 10 * double(rand())/double(RAND_MAX);
	goto_pose.y = 10 * double(rand())/double(RAND_MAX);
	//make sure our pose is greater than 1 to avoid clamping
	if (goto_pose.x < 1) {
		goto_pose.x = goto_pose.x + 1;
	}
	if (goto_pose.y < 1) {
		goto_pose.y = goto_pose.y + 1;
	}
	//max range of pi to pi
	goto_pose.theta = (2 * pi * double(rand())/double(RAND_MAX)) - pi;
	//variable to initiate a delay of 5 seconds
	int count = 0;
	//wait here to ensure the subscriber receives data
	while (ros::ok()) 
	{
		if (count >= 10) {
			//publish data
			ROS_INFO("\n\n\n******START PUBLISHING****************************\n");
			ROS_INFO_STREAM("Goal X: " << goto_pose.x);
			ROS_INFO_STREAM("Goal Y: " << goto_pose.y);
			ROS_INFO_STREAM("Goal Theta: " << goto_pose.theta);
			ROS_INFO("To publish new data, hit control c and re-run node.\n");
			ROS_INFO("\n\n\n**************************************************\n");
			//publish goal pose
			cmd_pose.publish(goto_pose);
		}
		else {
			count++;
		}
		//wait for loop
		loop_rate.sleep();
	}
	//just in case
	ros::shutdown();
	return 0;
}