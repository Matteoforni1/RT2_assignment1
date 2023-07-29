/**
*\file publisher.cpp
*\brief publisher of the second assignment
*\author Forni Matteo
*\version 1.0
*\date 21/04/2023
*
*\details
*
* Subscribes to : <BR>
*	/odom
*
* Publishes to : <BR>
*	/my_info
*
* 
* Description : 
*
* This node subscribes to the topic '/odom' to retrive information about the robot's current position and velocity.
* Then, this node publishes the information gathered on the topic '/my_info'.
*/




#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "assignment_2/info_robot.h"
#include <sstream>
#include <iostream>
#include <unistd.h>

// initializing variables
double posx; ///< X coordinate of current robot position.
double posy; ///< Y coordinate of current robot position.
double velx; ///< Current linear velocity of the robot.
double velz; ///< Current angular velocity of the robot.
ros::Publisher pub; ///< Publisher for this node

/**
*\brief callback function for the subscriber
*
*\param a message of type Odometry
*
*\return None
*
* This function is used to take information on the robot's current position and velocity.
* Then, this function publishes, using custom messages, the information just retrieved on the topic '/my_info'.
*
*/

void myCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	
	// we take the information we need from the topic
	posx = msg->pose.pose.position.x;
	posy = msg->pose.pose.position.y;
	velx = msg->twist.twist.linear.x;
	velz = msg->twist.twist.angular.z;
	ROS_INFO("[%f,%f,%f,%f]",posx,posy,velx,velz);
	assignment_2::info_robot info;
	
	// we publish the information obtained on the custom topic /info, using custom messages
	info.x = posx;
	info.y = posy;
	info.velx = velx;
	info.velz = velz;
	pub.publish(info);
}

/**
*\brief Main function of the node
*
*\param argc the number of arguments
*\param argv contains the arguments
*
*\return 0 as convention
*
* This function creates the subscriber and the publisher.
* Then, using the ros::spin method, it waits for new information to be published on the '/odom' topic.
*
*/

int main (int argc, char **argv) {
	
	// initializing the node
	ros::init(argc,argv,"publisher");
	ros::NodeHandle nh;
	
	// initializing the publisher and the subscriber
	pub = nh.advertise<assignment_2::info_robot>("/my_info",1);
	ros::Subscriber sub = nh.subscribe("odom",1,myCallback);
	
	// we set the 'spin rate'
	ros::Rate rate(1);
	
	assignment_2::info_robot info;
	rate.sleep();
	ros::spin();
}
