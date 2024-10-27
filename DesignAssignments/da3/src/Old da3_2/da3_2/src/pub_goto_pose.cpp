#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

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
	goto_pose.x = (rand()%2 == 0) ? goto_pose.x : 0 - goto_pose.x;
	goto_pose.y = 10 * double(rand())/double(RAND_MAX);
	goto_pose.y = (rand() > (RAND_MAX/2)) ? goto_pose.y : 0 - goto_pose.y;
	goto_pose.theta = 360 * double(rand())/double(RAND_MAX);

	//round position data to 2 decimals
	goto_pose.x = round(goto_pose.x*100.0)/100.0;
	goto_pose.y = round(goto_pose.y*100.0)/100.0;
	goto_pose.theta = round(goto_pose.theta*100.0)/100.0;

	//wait here to ensure the subscriber receives data
	while (ros::ok()) 
	{
		//publish data
		ROS_INFO("\n\n\n******START PUBLISHING****************************\n");
		ROS_INFO_STREAM("Goal X: " << goto_pose.x);
		ROS_INFO_STREAM("Goal Y: " << goto_pose.y);
		ROS_INFO_STREAM("Goal Theta: " << goto_pose.theta);
		ROS_INFO("To publish new data, hit control c and re-run node.\n");
		ROS_INFO("\n\n\n**************************************************\n");
		//publish goal pose
		cmd_pose.publish(goto_pose);
		//wait for callbacks
		ros::spinOnce();
		//wait for loop
		loop_rate.sleep();
	}
	//just in case
	ros::shutdown();
	return 0;
}