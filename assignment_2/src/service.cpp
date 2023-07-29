/**
*\file service.cpp
*\brief service of the second assignment
*\author Forni Matteo
*\version 1.0
*\date 21/04/2023
*
*\details
*
* Subscribes to : <BR>
*	/reaching_goal/result
*
* Services : <BR>
*	/result
*
* Description : 
*
* This node is the server side of the custom service implemented in the second assignment.
* This node subscribes to the '/reaching_goal/result' topic, in order to obtain information about the current status of the goal.
* When a goal is reached or cancelled, the number of goals reached or cancelled is raised by one unit.
* When the client calls for the service, these two numbers are passet to the client, so that it can prints them on the terminal.
*/




#include "ros/ros.h"
#include "assignment_2/info_robot.h"
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <assignment_2/PlanningActionGoal.h>
#include <assignment_2_2022/PlanningActionResult.h>
#include "assignment_2/Goal.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "actionlib_msgs/GoalStatus.h"

int n_reach = 0; ///< Number of goals reached.
int n_cancel = 0; ///< Number of goals cancelled.
int state; ///< Current state of the goal (each state corresponds to an integer).

/**
*\brief callback function for the subscriber to '/reaching_goal/result'
*
*\param a message of type PlanningActionResult
*
*\return None
*
* This function is used to retrieve the current state of the goal.
* If a goal has been reached/cancelled, the number of goals reached/cancelled is increased.
*
*/

void myCallback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg) {
	
	// we get the current state of the action
	state = msg->status.status;
	ROS_INFO("%d",state);
	
	// if the state is 2, a goal has been cancelled, so we increase of one unit the number of cancelled goals
	if(state == 2) {
		n_cancel += 1;
	}
	// if the state is 1, a goal has been reached, so we increase of one unit the number of reached goals
	else if(state == 3) {
		n_reach += 1;
	}
}

/**
*\brief callback function for the custom service '/result'
*
*\param a request of type Goal
*\param a response of type Goal
*
*\return Always True for convention
*
* This function is used to take information on the robot's current position and velocity.
* Then, this function computes the current average linear and angular speed of the robot and the distance between the robot and the goal.
*
*/

bool serviceCallback(assignment_2::Goal::Request &req, assignment_2::Goal::Response &res) {
	
	// we set as response the number of goals reached and cancelled
	res.reach = n_reach;
	res.canc = n_cancel;
	return true;
}

/**
*\brief main function for this node
*
*\param argc the number of arguments
*\param argv contains the arguments
*
*\return 0 as convention
*
* This function creates the subscribers and the server side of the custom service.
* Then, this function uses the ros::spin method to wait for the state of the goal to change, in order to perform the operation described in the service callback function.
*
*/

int main(int argc, char **argv) {
	
	//  initializing the node
	ros::init(argc,argv,"service");
	ros::NodeHandle nh;
	
	// initializing the custom service and the subscriber
	ros::ServiceServer service = nh.advertiseService("/result",serviceCallback);
	ros::Subscriber sub = nh.subscribe("reaching_goal/result",1,myCallback);
	
	ros::spin();
}
