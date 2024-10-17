#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

int main (int argc, char **argv)
{
	//initialize the publisher for pose
	ros::init(argc, argv, "pub_pose");
	ros::NodeHandle n;
	//topic name: goal_pose
	ros::Publisher pose_publisher = n.advertise<geometry_msgs::Pose2D>("goal_pose", 1000);
	//2 hertz
	ros::Rate loop_rate(2);

	//message pose_goal is of type Pose2D
	geometry_msgs::Pose2D goal_pose;

	//seed our random function
	srand(time(0));

	//publish one goal pose
	goal_pose.x = 10 * double(rand())/double(RAND_MAX);
	goal_pose.y = 10 * double(rand())/double(RAND_MAX);
	goal_pose.theta = 360 * double(rand())/double(RAND_MAX);

	//round position data to 2 decimals
	goal_pose.x = round(goal_pose.x*100.0)/100.0;
	goal_pose.y = round(goal_pose.y*100.0)/100.0;
	goal_pose.theta = round(goal_pose.theta*100.0)/100.0;

	//wait here to ensure the subscriber receives data
	while (ros::ok()) 
	{
		//publish data
		ROS_INFO("\n\n\n******START PUBLISHING****************************\n");
		ROS_INFO_STREAM("Goal X: " << goal_pose.x);
		ROS_INFO_STREAM("Goal Y: " << goal_pose.y);
		ROS_INFO_STREAM("Goal Theta: " << goal_pose.theta);
		ROS_INFO("To publish new data, hit control c and re-run node.\n");
		ROS_INFO("\n\n\n**************************************************\n");
		//publish goal pose
		pose_publisher.publish(goal_pose);
		//wait for callbacks
		ros::spinOnce();
		//wait for loop
		loop_rate.sleep();
	}
	//just in case
	ros::shutdown();
	return 0;
}