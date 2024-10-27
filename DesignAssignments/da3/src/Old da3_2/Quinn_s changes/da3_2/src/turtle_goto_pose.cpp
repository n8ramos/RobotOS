#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Pose2D.h>
//define our pi variable
	const double pi = 3.141592653589793238463;
//variables to publish
	//variable to store velocity data
	geometry_msgs::Twist tt_vel;
	double linearx, lineary, linearz, angularx, angulary, angularz;
	//variable to store pose data
	geometry_msgs::Pose2D pose_tt;
	//variable to store command velocity data
	geometry_msgs::Twist cmd_vel_tt;
    double x_goal, y_goal, th_goal;
//variables for math
	double distance = 0.0;
	double goalTime = 0.0;
//variable for initializing variables based upon received data
	bool initFlag = false;
//callback functions
	//callback function for receiving command velocities and driving tintin
	void callback_gotopose(const geometry_msgs::Pose2D& goto_pose);
	//callback function for receiving tintin pose and publishing data to topic
	void callback_pose(const turtlesim::Pose& pose);
	//callback function for receiving command velocities and driving tintin
	void callback_cmd_vel(const geometry_msgs::Twist& cmd_vel);
// calculate x distance and y disance
	float x_dist = 0.0;
	float y_dist = 0.0;
	// calculate target theta
	double target_theta = 0.0;

	// calculate target distance
	double target_dist = 0.0;

int main (int argc, char **argv) {	
	//initialize ros node to control turtle
	ros::init(argc, argv, "turtle_goto_pose");
	ros::NodeHandle n;
	//handle tintin velocity
		//subscriber to random command velocities
		ros::Subscriber cmd_sub = n.subscribe("goto_pose", 1, callback_gotopose);
		//publisher for tin tin velocity
		ros::Publisher vel_publisher = n.advertise<geometry_msgs::Twist>("tintin/cmd_vel", 100);
	//handle tintin pose
		//subscriber to tin tin pose
		ros::Subscriber read_tintin = n.subscribe("tintin/pose", 1, callback_pose);
		//publisher for pose data
		ros::Publisher pose_publisher = n.advertise<geometry_msgs::Pose2D>("pose_tt", 1);
	//handle tintin velocity
		//subscriber to tin tin command velocity
		ros::Subscriber read_tintin_vel = n.subscribe("tintin/cmd_vel", 1, callback_cmd_vel);
		//publisher for pose data
		ros::Publisher tt_cmdvel_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel_tt", 1);
	//500 hz
	ros::Rate loop_rate(500);
	
    //variables to calculate pos data
	double delta_x, delta_y, delta_th, delta_dist, dt;
	delta_x = delta_y = delta_th = delta_dist = dt = 0;

	// variables for time data
	ros::Time current_time, last_time;
	current_time = last_time = ros::Time::now();

	// variables to store velocity
	geometry_msgs::Twist vel_tt; 
	vel_tt.linear.x = 0;
	vel_tt.angular.z = 0;

    // flag for goal complete
	bool goal_reached = false;

	// phases for actions
	// 1 = change angle of approach (rotate turtle towards goal)
	// 2 = change position (move forward towards goal)
	// 3 = change orientation (face same direction as goal)
	int phase = 1; 
	
	// distance used for calculations
	double current_dist = 0;
    
    //begin operating node
	while (ros::ok())
	{
		ros::spinOnce();
		if (!goal_reached) // check if goal has been reached
		{
			switch (phase) {
				case 1:
					// change angle of approach
					if ((pose_tt.theta >= target_theta - 0.01) && (pose_tt.theta <= target_theta + 0.01)) {
						vel_tt.angular.z = 0.0;
						phase = 2;
					} else {
						vel_tt.angular.z = 0.1;
					}
					break;
				case 2:
					// change position
					if (current_dist < target_dist) {
						vel_tt.linear.x = 0.1;
					} else {
						vel_tt.linear.x = 0;
						phase = 3;
					}
					break;
				case 3:
					// change orientation
					if (pose_tt.theta != th_goal) {
						vel_tt.angular.z = 0.1;
					} else {
						vel_tt.angular.z = 0;
						goal_reached = true;
					}
					break;
				default:
					ROS_INFO("ERROR in PHASE\n");
					break;
			}

			
			// publish current data
			pose_publisher.publish(pose_tt);
			vel_publisher.publish(vel_tt);
			tt_cmdvel_publisher.publish(cmd_vel_tt);

			ROS_INFO_STREAM("Goal Theta: " << th_goal);
			ROS_INFO_STREAM("Goal Position: (" << x_goal << ", " << y_goal << ")");
			ROS_INFO_STREAM("Target Theta: " << target_theta);
			ROS_INFO_STREAM("Target Distance: " << target_dist);
			ROS_INFO_STREAM("Current Theta: " << pose_tt.theta);
			ROS_INFO_STREAM("Current Distance Traveled: " << current_dist);
			ROS_INFO_STREAM("Current Position: (" << pose_tt.x << ", " << pose_tt.y << ")\n");
			
			// calculate deltas
			current_time = ros::Time::now();
			dt = (current_time - last_time).toSec();

			delta_dist = vel_tt.linear.x * dt;

			// calculate new distance
			current_dist += delta_dist;

			//reset time
			last_time = current_time;
		} else {
			ROS_INFO_STREAM("Goal Theta: " << th_goal);
			ROS_INFO_STREAM("Goal Position: (" << x_goal << ", " << y_goal << ")");
			ROS_INFO_STREAM("Current Theta: " << pose_tt.theta);
			ROS_INFO_STREAM("Current Position: (" << pose_tt.x << ", " << pose_tt.y << ")\n");
			ROS_INFO("Goal reached!\n\n");
		}
		loop_rate.sleep();
	}

	//just in case
	ros::shutdown();
	return 0;
}
//callback function for receiving goal pose for tintin
void callback_gotopose(const geometry_msgs::Pose2D& goto_pose) {
	//read data into buffer variables (not the same variables that are published)
	x_goal = goto_pose.x;
    y_goal = goto_pose.y;
    th_goal = goto_pose.theta;
    if (x_goal > 0.0 && !initFlag) {
    	x_dist = x_goal - pose_tt.x;
		y_dist = y_goal - pose_tt.y;
		// calculate target theta
		target_theta = atan2(y_dist, x_dist);
		// calculate target distance
		target_dist = sqrt(x_dist*x_dist + y_dist*y_dist);
    	initFlag = true;
    }
    // update done flag
}
//callback function for receiving tintin pose and publishing data to topic
void callback_pose(const turtlesim::Pose& pose) {
	//read data directly into published variables
	pose_tt.x = pose.x;
	pose_tt.y = pose.y;
	pose_tt.theta = pose.theta;
}
void callback_cmd_vel(const geometry_msgs::Twist& cmd_vel) {
	//read data directly from tintin
	cmd_vel_tt.linear.x = cmd_vel.linear.x;
	cmd_vel_tt.linear.y = cmd_vel.linear.y;
	cmd_vel_tt.linear.z = cmd_vel.linear.z;
	cmd_vel_tt.angular.x = cmd_vel.angular.x;
	cmd_vel_tt.angular.y = cmd_vel.angular.y;
	cmd_vel_tt.angular.z = cmd_vel.angular.z;
}