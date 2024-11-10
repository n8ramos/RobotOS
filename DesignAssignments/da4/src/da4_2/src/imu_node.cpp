#include <ros/ros.h>
//for RPY Node
#include <std_msgs/Float32MultiArray.h>
//for IMU Node
#include <sensor_msgs/Imu.h>
//for Quaternion conversion
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
//For rviz broadcasting
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

//Variable Declarations
	//message for imu_pub
	sensor_msgs::Imu imu_msg;
	//rpy to quaternion conversion
	tf2::Quaternion q;
	//flag for data
	bool new_data = false;
//Function Declarations
	//function for calling back the STM32F103RCT6 /rpy node
	void callback_rpy(const std_msgs::Float32MultiArray& rpy);

int main (int argc, char **argv) {
	//initialize the rosnode
		ros::init(argc, argv, "imu_node");
	//nodehandler
		ros::NodeHandle n;
	//topic /rpy
		ros::Subscriber rpy_sub = n.subscribe("rpy", 1, callback_rpy);
	//topc /imu
		ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
	//500 hz 
		ros::Rate loop_rate(500);
	//For RVIZ Broadcasting
		static tf2_ros::TransformBroadcaster br;
		geometry_msgs::TransformStamped transformStamped;
			transformStamped.header.frame_id = "map";
		    transformStamped.child_frame_id = "imu_link";
		    transformStamped.transform.translation.x = 0.0;
			transformStamped.transform.translation.y = 0.0;
			transformStamped.transform.translation.z = 0.0;
		    
	//begin operation
	while (ros::ok()) {
		//receive subscriber data
			ros::spinOnce();
		//verify that data has changed
			if (new_data) {
			//publish IMU data
				imu_pub.publish(imu_msg);
			//Set transformation values
			    transformStamped.header.stamp = imu_msg.header.stamp;
			    transformStamped.transform.rotation = imu_msg.orientation;
			//publish transformation values
				br.sendTransform(transformStamped);
			//flip data flag
				new_data = false;
			}
		loop_rate.sleep();
	}
	ros::shutdown();
	return 0;
}

//function for calling back the STM32F103RCT6 /rpy node
void callback_rpy(const std_msgs::Float32MultiArray& rpy) {
	// Convert degrees to radians
    float roll_rad = rpy.data[0] * M_PI / 180.0;
    float pitch_rad = rpy.data[1] * M_PI / 180.0;
    float yaw_rad = rpy.data[2] * M_PI / 180.0;
	// Convert RPY angles from degrees to radians if necessary
    double roll = rpy.data[0];  // Assuming values are already in radians; convert if in degrees
    double pitch = rpy.data[1];
    double yaw = rpy.data[2];
    
    q.setRPY(roll_rad, pitch_rad, yaw_rad); // Convert RPY to Quaternion

    // Set IMU message orientation directly
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    
    // Update the frame ID and timestamp for each received message
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_link";  // Set this only if it changes dynamically

    //Indicate that we have new data
    new_data = true;
}