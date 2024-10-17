#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
// #include <glm/glm.hpp> // n8

//global variable init
double x_goal, y_goal, theta_goal;

// flag for callback
bool done;

//callback for goal_pose subscriber
void callback_goalPose(const geometry_msgs::Pose2D& goal_pose);

int main (int argc, char **argv)
{
	//initialize controller
	ros::init(argc, argv, "ctrl_pose");
	ros::NodeHandle n;
	//subscriber for goal_pose
	ros::Subscriber pose_subscriber = n.subscribe("goal_pose", 1, callback_goalPose);
	//publisher for current pose
	ros::Publisher pose_publisher = n.advertise<geometry_msgs::Pose2D>("current_pose", 1);
	//publisher for current velocity
	ros::Publisher cmdVel_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	//freq rate (Hz)
	ros::Rate loop_rate(500);
	ros::Rate display_rate(10);

	//variables to calculate pos data
	double delta_x, delta_y, delta_th, delta_dist, dt;
	delta_x = delta_y = delta_th = delta_dist = dt = 0;

	// variables for time data
	ros::Time current_time, last_time;
	current_time = last_time = ros::Time::now();

	// variables to store velocity
	geometry_msgs::Twist current_vel; 
	current_vel.linear.x = 0;
	current_vel.angular.z = 0;

	// variables to store position
	geometry_msgs::Pose2D current_pose; 
	current_pose.x = 0; 
	current_pose.y = 0; 
	current_pose.theta = 0;
	double x, y, th;
	x = y = th = 0;

	// wait for a goal pose callback
	done = false;
	while (!done) {
		ros::spinOnce();
	}
	// calculate x distance and y disance
	float x_dist = x_goal - current_pose.x;
	float y_dist = y_goal - current_pose.y;
	// calculate target theta
	float y_over_x = (y_dist / x_dist);
	double target_theta = atan(y_over_x);
	
	// convert theta from rad to deg
	target_theta = target_theta * (180.0/3.141592653589793238463);
	// round target theta to 2 decimals
	target_theta = round(target_theta*100.0) / 100.0;

	// calculate target distance
	double target_dist = sqrt(x_dist*x_dist + y_dist*y_dist);

	// round distance to 2 decimals
	target_dist = round(target_dist*100.0) / 100.0;

	// flag for goal complete
	bool goal_reached = false;

	// phases for actions
	// 1 = change angle of approach (rotate robot towards goal)
	// 2 = change position (move forward towards goal)
	// 3 = change orientation (face same direction as goal)
	int phase = 1; 
	
	// distance used for calculations
	double current_dist = 0;
	while (ros::ok())
	{
		if (!goal_reached) // check if goal has been reached
		{
			switch (phase) {
				case 1:
					// change angle of approach
					if (current_pose.theta != target_theta) {
						current_vel.angular.z = 0.1;
					} else {
						current_vel.angular.z = 0;
						phase = 2;
					}
					break;
				case 2:
					// change position
					if ((round(current_dist*100.0)/100.0) != target_dist) {
						current_vel.linear.x = 0.1;
					} else {
						current_vel.linear.x = 0;
						phase = 3;
					}
					break;
				case 3:
					// change orientation
					if (current_pose.theta != theta_goal) {
						current_vel.angular.z = 0.1;
					} else {
						current_vel.angular.z = 0;
						goal_reached = true;
					}
					break;
				default:
					ROS_INFO("ERROR in PHASE\n");
					break;
			}

			
			// publish current data
			pose_publisher.publish(current_pose);
			cmdVel_publisher.publish(current_vel);
			ROS_INFO_STREAM("Goal Theta: " << theta_goal);
			ROS_INFO_STREAM("Goal Position: (" << x_goal << ", " << y_goal << ")");
			ROS_INFO_STREAM("Target Theta: " << target_theta);
			ROS_INFO_STREAM("Target Distance: " << target_dist);
			ROS_INFO_STREAM("Current Theta: " << current_pose.theta);
			ROS_INFO_STREAM("Current Distance Traveled: " << current_dist);
			ROS_INFO_STREAM("Current Position: (" << current_pose.x << ", " << current_pose.y << ")\n");
			
			// calculate deltas
			current_time = ros::Time::now();
			dt = (current_time - last_time).toSec();
			delta_th = current_vel.angular.z * dt;

			delta_dist = current_vel.linear.x * dt;

			// calculate new distance
			current_dist += delta_dist;

			// calculate new pos
			x = (phase == 2) ? current_dist * cos(th) : x;
			y = (phase == 2) ? current_dist * sin(th) : y;
			th += delta_th;

			if (th >= 2*3.141592653589793238463) {th = 0;}
			current_pose.x = round(x*100.0)/100.0;
			current_pose.y = round(y*100.0)/100.0;
			current_pose.theta = round((th * (180.0/3.141592653589793238463)*100.0))/100.0;

			last_time = current_time;
		} else {
			ROS_INFO_STREAM("Goal Theta: " << theta_goal);
			ROS_INFO_STREAM("Goal Position: (" << x_goal << ", " << y_goal << ")");
			ROS_INFO_STREAM("Current Theta: " << current_pose.theta);
			ROS_INFO_STREAM("Current Position: (" << current_pose.x << ", " << current_pose.y << ")\n");
			ROS_INFO("Goal reached!\n\n");
		}
		loop_rate.sleep();
	}

	//just in case
	ros::shutdown();
	return 0;
}

//callback for goal_pose subscriber - edits by n8
void callback_goalPose(const geometry_msgs::Pose2D& goal_pose)
{
	//store goal positions
	x_goal = goal_pose.x;
	y_goal = goal_pose.y;
	
	//store goal angle
	theta_goal = goal_pose.theta;

	// update done flag
	done = true;
	// output message
	ROS_INFO("Received the following goal pose values: x = %f, y = %f, theta = %f", x_goal, y_goal, theta_goal);
}