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
	geometry_msgs::Pose2D tt_pose;
//variables for math
	double distance = 0.0;
	double goalTime = 0.0;
//variables for commands
	bool driveFlag = false;
	bool turnFlag = false;
	int squareSide = 1;
	bool initFlag = true;
	bool stopFlag = false;
//callback functions
	//callback function for receiving command velocities and driving tintin
	void callback_cmd_vel(const geometry_msgs::Twist& cmd_vel);
	//callback function for receiving tintin pose and publishing data to topic
	void callback_pose(const turtlesim::Pose& pose);
	//callback function for our timer
	void callback_timer(const ros::TimerEvent&);
//turning function (eliminate repetitive code)
	void stopTurn(ros::Timer timer, int newSquareSide);
int main (int argc, char **argv) {	
	//initialize ros node to control turtle
	ros::init(argc, argv, "turtle_motion");
	ros::NodeHandle n;
	//handle tintin velocity
		//subscriber to random command velocities
		ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1, callback_cmd_vel);
		//publisher for tin tin velocity
		ros::Publisher drive_tintin = n.advertise<geometry_msgs::Twist>("tintin/cmd_vel", 100);
	//handle tintin pose
		//subscriber to tin tin pose
		ros::Subscriber read_tintin = n.subscribe("tintin/pose", 1, callback_pose);
		//publisher for pose data
		ros::Publisher pose_publisher = n.advertise<geometry_msgs::Pose2D>("pose_tt", 1);
	//500 hz
	ros::Rate loop_rate(500);
	//create timer
		ros::Timer timer1 = n.createTimer(ros::Duration(0.0), callback_timer, true);
	//begin operating node
	while(ros::ok())
	{
		//update data from callback
		ros::spinOnce();
		//are we stopping the timer?
		if (stopFlag) {
			//if we are getting ready to turn, stop the timer
			timer1.stop();
		}
		//initiate timer - this should only trigger once
		if (initFlag) {
			if (goalTime > 0.0) {
				//determine time of driving
				timer1.setPeriod(ros::Duration(goalTime));
				//begin timer
				timer1.start();
				//flip flag so we never run this if block again
				initFlag = false;
				//we want to start driving
				driveFlag = true;
			}
		}
		//after the first initialization, this will run everytime
		if (!initFlag) {
			//driving
			if (driveFlag) {
				//pass velocity values
				tt_vel.linear.x = linearx;
				tt_vel.linear.y = lineary;
				tt_vel.linear.z = linearz;
				tt_vel.angular.x = angularx;
				tt_vel.angular.y = angulary;
				tt_vel.angular.z = angularz;
			}
			//turning
			else if (turnFlag) {
				//stop driving and start turning counter clockwise
				tt_vel.linear.x = 0.0;
				tt_vel.linear.y = lineary;
				tt_vel.linear.z = linearz;
				tt_vel.angular.x = angularx;
				tt_vel.angular.y = angulary;
				tt_vel.angular.z = 0.1;
				switch (squareSide) {
					//turning toward +y direction
					case 1:
						//if we have reached the goal theta for +y axis
						if (tt_pose.theta >= pi / 2) {
							//call stopTurn
							stopTurn(timer1, 2);
						}
						break;
					//turning toward -x direction
					case 2:
						//if we have reached the goal theta for -x axis
						if (tt_pose.theta >= (-1 * pi) && tt_pose.theta < -3)  {
							//call stopTurn
							stopTurn(timer1, 3);
						}
						break;
					//turning toward -y direction
					case 3:
						//if we have reached the goal theta for -y axis
						if (tt_pose.theta >= (-1 * pi) / 2  && tt_pose.theta < -1) {
							//call stopTurn
							stopTurn(timer1, 4);
						}
						break;
					//turning toward +x direction
					case 4:
						//if we have reached the goal theta for +x axis
						if (tt_pose.theta >= 0.0) {
							//call stopTurn
							stopTurn(timer1, 1);
						}
						break;
					default:
						//this should never happen
						break;
				}
			}
		}
		//publish values
		drive_tintin.publish(tt_vel);
		pose_publisher.publish(tt_pose);
		//delay for remainder of frequency
		loop_rate.sleep();
	}
	//terminate node
	ros::shutdown();
	return 0;
}
//callback function for receiving command velocities and driving tintin
void callback_cmd_vel(const geometry_msgs::Twist& cmd_vel) {
	//read data into buffer variables (not the same variables that are published)
	linearx = cmd_vel.linear.x;
	//linear.y is reserved for desired distance
	lineary = 0.0;
	distance = cmd_vel.linear.y;
	linearz = cmd_vel.linear.z;
	angularx = cmd_vel.angular.x;
	angulary = cmd_vel.angular.y;
	angularz = cmd_vel.angular.z;

	//calculate goal time
	goalTime = distance/linearx;
}
//callback function for receiving tintin pose and publishing data to topic
void callback_pose(const turtlesim::Pose& pose) {
	//read data directly into published variables
	tt_pose.x = pose.x;
	tt_pose.y = pose.y;
	tt_pose.theta = pose.theta;
}
//callback function for our timer
void callback_timer(const ros::TimerEvent&) {
	//if we're not initializing the timer
	if (!initFlag) {
		//stop the timer
		stopFlag = true;
		//stop driving
		driveFlag = false;
		//start turning
		turnFlag = true;
	}
}
//function to eliminate repetitive code in our turning block
void stopTurn (ros::Timer timer, int newSquareSide) {
	//stop turning and pass in velocity/distance values
	tt_vel.linear.x = linearx;
	tt_vel.linear.y = lineary;
	tt_vel.linear.z = linearz;
	tt_vel.angular.x = angularx;
	tt_vel.angular.y = angulary;
	tt_vel.angular.z = 0.0;
	//indicate the new side of the square we're on
	squareSide = newSquareSide;
	//stop turning
	turnFlag = false;
	//start driving
	driveFlag = true;
	//don't stop the timer
	stopFlag = false;
	//restart timer
	timer.setPeriod(ros::Duration(goalTime));
	timer.start();
}