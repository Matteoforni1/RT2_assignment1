/**
*\file subscriber.cpp
*\brief subscriber of the second assignment
*\author Forni Matteo
*\version 1.0
*\date 21/04/2023
*
*\details
*
* Subscribes to : <BR>
*	/reaching_goal/goal	<BR>
*	/my_info
*
* Parameters : <BR>
*	publish_rate
*
* Description : 
*
* This node subscribes to the topic '/my_info' to retrive information about the robot's current position and velocity.
* Then, it computes the distance from the goal and the average speed.
*/




#include "ros/ros.h"
#include "assignment_2/info_robot.h"
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <assignment_2/PlanningActionGoal.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

double px_r; ///< X coordinate of current robot position.
double py_r; ///< Y coordinate of current robot position.
double px_g; ///< X coordinate of current goal position.
double py_g; ///< Y coordinate of current goal position.
double vx; ///< Current robot linear velocity.
double vz; ///< Current robot angular velocity.
double gdx; ///< X coordinate of current distance between robot and goal's position.
double gdy; ///< Y coordinate of current distance between robot and goal's position.
double avg_speed_x = 0; ///< Current robot average linear speed.
double avg_speed_z = 0; ///< Current robot average angular speed.
int n_sub = 0; ///< Number of subscription to '/my_info'.
double publish_rate; ///< Rate at which the node publish information.

/**
*\brief callback function for the subscriber to '/my_info'
*
*\param a message of type info_robot
*
*\return None
*
* This function is used to take information on the robot's current position and velocity.
* Then, this function computes the current average linear and angular speed of the robot and the distance between the robot and the goal.
*
*/

void myCallback(const assignment_2::info_robot::ConstPtr& msg) {
	
	// we take the information from the custom message
	px_r = msg->x;
	py_r = msg->y;
	vx = msg->velx;
	vz = msg->velz;
	
	// we compute the average speeds
	avg_speed_x *= n_sub;
	avg_speed_z *= n_sub;
	n_sub++;
	avg_speed_x += vx;
	avg_speed_z += vz;
	avg_speed_x /= n_sub;
	avg_speed_z /= n_sub;
	
	// the average velocities are printed
	ROS_INFO("AVERAGE VELOCITIES: [%f,%f]",avg_speed_x,avg_speed_z);
}

/**
*\brief callback function for the subscriber to '/reaching_goal/goal'
*
*\param a message of type PlanningActionGoal
*
*\return None
*
* This function is used to take information on the goal's current position.
*
*/

void myCallback2(const assignment_2::PlanningActionGoal::ConstPtr& msg) {
	
	// we get information about the robot's current position
	px_g = msg->goal.target_pose.pose.position.x;
	py_g = msg->goal.target_pose.pose.position.y;
}

/**
*\brief function to compute the distance between two 2D points
*
*\param None
*
*\return None
*
* This function is used to compute the euclidean distance among two points and print it on the terminal.
*
*/

void distance() {
	// we compute the distances between the robot and the target both on the x axis and the y axis
	gdx = abs(px_r-px_g);
	gdy = abs(py_r-py_g);
		
	// the two distances are printed
	ROS_INFO("DISTANCE FROM TARGET: [%f,%f]",gdx,gdy);
}

/**
*\brief main function for this node
*
*\param argc the number of arguments
*\param argv contains the arguments
*
*\return 0 as convention
*
* This function creates the subscribers and gets the parameter publish_rate.
* Then, this function calls the functrion distance(), in order to publish information on the terminal.
*
*/

int main (int argc, char **argv) {
	
	//  initializing the node
	ros::init(argc,argv,"subscriber");
	ros::NodeHandle nh;
	// get frequency from the launch file and setting the rate to that parameter
	ros::param::get("publish_rate", publish_rate);
	ros::Rate rate(publish_rate);
	
	// initializing the two subscribers
	ros::Subscriber sub1 = nh.subscribe("my_info",1,myCallback);
	ros::Subscriber sub2 = nh.subscribe("reaching_goal/goal",1,myCallback2);
	
	// while the program is running...
	while (ros::ok()) {
		
		// function for computing the distance
		distance();
		rate.sleep();
		ros::spinOnce();
	}
}
